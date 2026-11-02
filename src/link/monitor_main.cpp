// link_monitor: MAVLink link-health monitor using our own parser (no MAVLink library, no
// MAVSDK). Listens on a UDP port, reports frame/CRC/loss statistics once per second, and
// warns when the autopilot heartbeat stops.
//
// Usage:   link_monitor [--port 14550] [--out link_stats.csv]
//
// CSV columns (one row per second, counters cumulative since start):
//   t_s,frames_ok,crc_errors,unknown_msgid,bad_header,lost,loss_ratio,rate_hz,heartbeat_age_s,
//   reordered
// heartbeat_age_s is -1 until the first autopilot HEARTBEAT arrives.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "asl/link/crc_extra.h"
#include "asl/link/link_stats.h"
#include "asl/link/mavlink_parser.h"

namespace {

constexpr std::uint32_t kMsgHeartbeat = 0;
constexpr std::uint8_t kCompAutopilot = 1;  // MAV_COMP_ID_AUTOPILOT1
constexpr double kHeartbeatTimeoutS = 3.0;
constexpr auto kReportPeriod = std::chrono::seconds(1);

std::atomic<bool> g_stop{false};

void on_sigint(int) { g_stop = true; }

// RAII owner of a bound UDP socket. The file descriptor is closed in the destructor, so no
// code path (including exceptions) can leak it.
class UdpSocket {
public:
    UdpSocket(std::uint16_t port, int timeout_ms) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            throw std::system_error(errno, std::generic_category(), "socket");
        }
        timeval tv{};
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        // Receive timeout: recv() returns at least every timeout_ms, so the 1 Hz report and
        // the heartbeat watchdog keep running even when no packets arrive.
        if (::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            const int err = errno;
            ::close(fd_);
            throw std::system_error(err, std::generic_category(), "setsockopt(SO_RCVTIMEO)");
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            const int err = errno;
            ::close(fd_);
            throw std::system_error(err, std::generic_category(),
                                    "bind to UDP port " + std::to_string(port) +
                                        " (is QGroundControl or another monitor running?)");
        }
    }
    ~UdpSocket() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Returns bytes received, or 0 on timeout / interrupted by a signal.
    std::size_t receive(std::uint8_t* buf, std::size_t cap) {
        const ssize_t n = ::recvfrom(fd_, buf, cap, 0, nullptr, nullptr);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return 0;
            }
            throw std::system_error(errno, std::generic_category(), "recvfrom");
        }
        return static_cast<std::size_t>(n);
    }

private:
    int fd_ = -1;
};

struct Args {
    std::uint16_t port = 14550;
    std::string out = "link_stats.csv";
};

bool parse_args(int argc, char** argv, Args& args) {
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string flag = argv[i];
        const std::string value = argv[i + 1];
        if (flag == "--port") {
            const int p = std::stoi(value);
            if (p <= 0 || p > 65535) {
                return false;
            }
            args.port = static_cast<std::uint16_t>(p);
        } else if (flag == "--out") {
            args.out = value;
        } else {
            return false;
        }
    }
    return argc % 2 == 1;  // flags come in pairs
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        if (!parse_args(argc, argv, args)) {
            throw std::invalid_argument("bad arguments");
        }
    } catch (const std::exception&) {
        std::cerr << "Usage: " << argv[0] << " [--port 14550] [--out link_stats.csv]\n";
        return 1;
    }

    // sigaction WITHOUT SA_RESTART: Ctrl-C interrupts a blocked recvfrom (EINTR) at once.
    struct sigaction sa {};
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    const std::filesystem::path out_path(args.out);
    if (out_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(out_path.parent_path(), ec);
    }
    std::ofstream csv(out_path);
    if (!csv) {
        std::cerr << "ERROR: cannot write " << args.out << "\n";
        return 1;
    }
    csv << "t_s,frames_ok,crc_errors,unknown_msgid,bad_header,lost,loss_ratio,rate_hz,"
           "heartbeat_age_s,reordered\n";

    std::unique_ptr<UdpSocket> sock;
    try {
        sock = std::make_unique<UdpSocket>(args.port, 200);
    } catch (const std::system_error& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
    std::cout << "Listening on UDP " << args.port << ", writing " << args.out << "\n";

    asl::link::MavlinkParser parser;
    asl::link::LinkStats link;
    std::map<std::uint32_t, std::uint64_t> per_msgid;  // for the exit summary
    std::array<std::uint8_t, 2048> buf{};  // one UDP datagram from PX4 is well below this

    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    auto next_report = t0 + kReportPeriod;
    auto last_report = t0;
    std::uint64_t frames_at_last_report = 0;
    bool have_heartbeat = false;
    Clock::time_point last_heartbeat{};

    while (!g_stop) {
        const std::size_t n = sock->receive(buf.data(), buf.size());
        if (n > 0) {
            for (const asl::link::Frame& f : parser.push(buf.data(), n)) {
                link.on_frame(f.sysid, f.compid, f.seq);
                ++per_msgid[f.msgid];
                if (f.msgid == kMsgHeartbeat && f.compid == kCompAutopilot) {
                    have_heartbeat = true;
                    last_heartbeat = Clock::now();
                }
            }
        }

        const auto now = Clock::now();
        if (now < next_report) {
            continue;
        }
        const auto& s = parser.stats();
        const double dt = std::chrono::duration<double>(now - last_report).count();
        const double rate = static_cast<double>(s.frames_ok - frames_at_last_report) / dt;
        const double hb_age =
            have_heartbeat ? std::chrono::duration<double>(now - last_heartbeat).count() : -1.0;
        const double t_s = std::chrono::duration<double>(now - t0).count();

        csv << std::fixed << std::setprecision(1) << t_s << ',' << s.frames_ok << ','
            << s.crc_errors << ',' << s.unknown_msgid << ',' << s.bad_header << ',' << link.lost()
            << ',' << std::setprecision(5) << link.loss_ratio() << ',' << std::setprecision(1)
            << rate << ',' << std::setprecision(2) << hb_age << ',' << link.reordered() << '\n';
        csv.flush();  // one small write per second; keeps the file current if we are killed

        std::cout << std::fixed << std::setprecision(0) << "t=" << t_s << "s  frames "
                  << s.frames_ok << "  rate " << rate << "/s  crc_err " << s.crc_errors
                  << "  lost " << link.lost() << " (" << std::setprecision(2)
                  << 100.0 * link.loss_ratio() << "%)  hb_age "
                  << (have_heartbeat ? std::to_string(hb_age).substr(0, 4) + "s" : "never");
        if (!have_heartbeat || hb_age > kHeartbeatTimeoutS) {
            std::cout << "  WARNING: no autopilot HEARTBEAT for > " << kHeartbeatTimeoutS << " s";
        }
        std::cout << "\n";

        last_report = now;
        frames_at_last_report = s.frames_ok;
        next_report += kReportPeriod;
        if (next_report < now) {
            next_report = now + kReportPeriod;  // don't burst if we were stalled
        }
    }

    // ---- Clean shutdown: summary, then the destructors close the socket and the file ----
    const auto& s = parser.stats();
    std::cout << "\nStopped. frames_ok " << s.frames_ok << ", crc_errors " << s.crc_errors
              << ", unknown_msgid " << s.unknown_msgid << ", bad_header " << s.bad_header
              << ", bytes_skipped " << s.bytes_skipped << ", lost " << link.lost() << " ("
              << std::setprecision(3) << 100.0 * link.loss_ratio() << "%), reordered "
              << link.reordered() << "\n";
    std::vector<std::pair<std::uint64_t, std::uint32_t>> top;
    for (const auto& [id, count] : per_msgid) top.emplace_back(count, id);
    std::sort(top.rbegin(), top.rend());
    std::cout << "Top messages:\n";
    for (std::size_t i = 0; i < top.size() && i < 8; ++i) {
        const auto* info = asl::link::find_msg_info(top[i].second);
        std::cout << "  " << std::setw(6) << top[i].first << "  " << (info ? info->name : "?")
                  << " (" << top[i].second << ")\n";
    }
    return 0;
}
