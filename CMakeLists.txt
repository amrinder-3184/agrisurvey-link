cmake_minimum_required(VERSION 3.16)
project(agrisurvey_link VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Language and build settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # lets VS Code / clangd find headers

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(ASL_BUILD_TESTS "Build the GoogleTest unit tests" ON)
option(ASL_SANITIZE    "Build with AddressSanitizer + UndefinedBehaviorSanitizer" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(ASL_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

# ---------------------------------------------------------------------------
# Core libraries (no MAVSDK dependency, so they are unit-testable anywhere)
# ---------------------------------------------------------------------------
# asl_core: geometry, survey planning, payload trigger      (phases 2-3)
add_library(asl_core
  src/core/geo.cpp
  src/core/survey_planner.cpp
  src/core/trigger.cpp
  src/core/config.cpp
  src/core/capture_queue.cpp
  src/core/geotag_logger.cpp
  src/core/event_log.cpp
)
target_include_directories(asl_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(Threads REQUIRED)                 # std::thread / condition_variable
target_link_libraries(asl_core PUBLIC Threads::Threads)

# asl_link: MAVLink CRC, frame parser, link statistics     (phase 4)
add_library(asl_link
  src/link/crc.cpp
  src/link/mavlink_parser.cpp
  src/link/link_stats.cpp
)
target_include_directories(asl_link PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# link_monitor: UDP link-health monitor on our own parser (phase 4). No MAVSDK needed.
add_executable(link_monitor src/link/monitor_main.cpp)
target_link_libraries(link_monitor PRIVATE asl_link)

# ---------------------------------------------------------------------------
# Executables that talk to PX4 through MAVSDK (built only if MAVSDK is installed)
# ---------------------------------------------------------------------------
find_package(MAVSDK QUIET)
if(MAVSDK_FOUND)
  message(STATUS "MAVSDK found: building mission executables")
  add_executable(hello_drone src/mission/hello_drone.cpp)       # phase 1
  target_link_libraries(hello_drone PRIVATE MAVSDK::mavsdk)
  add_executable(mission_node src/mission/mission_node.cpp)     # phase 2
  target_link_libraries(mission_node PRIVATE asl_core MAVSDK::mavsdk)
else()
  message(STATUS "MAVSDK not found: skipping mission executables (see docs/SETUP.md, step 4)")
endif()

# ---------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------
if(ASL_BUILD_TESTS)
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    message(STATUS "GoogleTest not installed: fetching v1.15.2")
    include(FetchContent)
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        v1.15.2)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  enable_testing()
  add_executable(asl_tests
    tests/test_geo.cpp
    tests/test_survey_planner.cpp
    tests/test_trigger.cpp
    tests/test_crc.cpp
    tests/test_mavlink_parser.cpp
    tests/test_link_stats.cpp
    tests/test_config.cpp
    tests/test_capture_queue.cpp
    tests/test_synthetic_field.cpp
    tests/test_geotag_logger.cpp
    tests/test_parser_robustness.cpp
    tests/test_link_reorder.cpp
    tests/test_event_log.cpp
  )
  target_link_libraries(asl_tests PRIVATE asl_core asl_link GTest::gtest_main)
  # Lets tests load the real files in config/ regardless of the working directory.
  target_compile_definitions(asl_tests PRIVATE ASL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  include(GoogleTest)
  gtest_discover_tests(asl_tests)
endif()
