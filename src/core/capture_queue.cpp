// Spec: include/asl/capture_queue.h. Tests: tests/test_capture_queue.cpp.
#include "asl/capture_queue.h"

#include <stdexcept>

namespace asl {

CaptureQueue::CaptureQueue(std::size_t capacity) {
    if (capacity == 0) {
        throw std::invalid_argument("CaptureQueue: capacity must be > 0");
    }
    buffer_.resize(capacity);  // the only allocation this class ever makes
}

bool CaptureQueue::push(const Capture& item) {
    bool no_drop = true;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) {
            return false;
        }
        if (count_ == buffer_.size()) {
            // Full: the oldest slot becomes the newest. head_ moves to the next-oldest.
            buffer_[head_] = item;
            head_ = (head_ + 1) % buffer_.size();
            ++dropped_;
            no_drop = false;
        } else {
            buffer_[(head_ + count_) % buffer_.size()] = item;
            ++count_;
        }
    }
    // Notify after unlocking, so the woken consumer does not immediately block on the mutex.
    not_empty_.notify_one();
    return no_drop;
}

bool CaptureQueue::pop_wait(Capture& out, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    // The predicate form re-checks the condition, so spurious wake-ups are handled.
    not_empty_.wait_for(lock, timeout, [this] { return count_ > 0 || closed_; });
    if (count_ == 0) {
        return false;  // timeout, or closed and fully drained
    }
    out = buffer_[head_];
    head_ = (head_ + 1) % buffer_.size();
    --count_;
    return true;
}

void CaptureQueue::close() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
    }
    not_empty_.notify_all();
}

std::size_t CaptureQueue::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
}

std::uint64_t CaptureQueue::dropped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
}

bool CaptureQueue::closed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
}

}  // namespace asl
