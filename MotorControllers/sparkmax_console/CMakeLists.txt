cmake_minimum_required(VERSION 3.21)
project(sparkmax_console CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# std::thread needs pthread on Linux, winpthread on MinGW, and nothing extra
# on MSVC — Threads::Threads resolves to the right thing on each.
find_package(Threads REQUIRED)

add_executable(sparkmax_console
    src/main.cpp
    src/serial_port.cpp
)

target_include_directories(sparkmax_console PRIVATE src)
target_link_libraries(sparkmax_console PRIVATE Threads::Threads)

if(WIN32)
    target_link_libraries(sparkmax_console PRIVATE setupapi)
endif()

if(MSVC)
    target_compile_options(sparkmax_console PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(sparkmax_console PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Build:
#   VS Code:  install the "CMake Tools" extension, then pick a preset
#             from the status bar (windows-msvc-debug, linux-debug, ...).
#   CLI:      cmake --preset windows-msvc-debug
#             cmake --build --preset windows-msvc-debug
#             build/windows-msvc-debug/sparkmax_console.exe
