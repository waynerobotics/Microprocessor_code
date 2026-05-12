// sparkmax_console
// ----------------
// Standalone cross-platform console controller for a single REV SPARK MAX
// motor controller over USB CDC.  Sends a heartbeat every 50 ms in the
// background and reads numeric setpoints (0..50 rotations) from stdin.
//
// Usage:
//   sparkmax_console [--port <name>] [--id <1..62>]
//
// Without --port, the app auto-discovers a SPARK MAX by USB VID/PID.

#include "serial_port.hpp"
#include "spark_protocol.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using namespace spark;
using namespace std::chrono_literals;

namespace {

constexpr float MIN_SETPOINT = 0.0f;
constexpr float MAX_SETPOINT = 50.0f;
constexpr int   HEARTBEAT_MS = 50;

std::atomic<bool> g_stop{false};
std::mutex        g_port_mutex;
SerialPort        g_port;
uint32_t          g_device_id = 1;

void heartbeat_thread() {
    const auto interval = std::chrono::milliseconds(HEARTBEAT_MS);
    while (!g_stop.load()) {
        {
            std::lock_guard<std::mutex> lk(g_port_mutex);
            if (g_port.is_open()) {
                Packet hb = Packet::heartbeat();
                if (!g_port.write_all(hb.bytes, PACKET_BYTES)) {
                    std::cerr << "[heartbeat] write failed — closing port\n";
                    g_port.close();
                }
            }
        }
        std::this_thread::sleep_for(interval);
    }
}

bool send_setpoint(float rotations) {
    std::lock_guard<std::mutex> lk(g_port_mutex);
    if (!g_port.is_open()) {
        std::cerr << "[tx] port not open — drop setpoint\n";
        return false;
    }
    Packet p = Packet::position_setpoint(rotations, g_device_id);
    if (!g_port.write_all(p.bytes, PACKET_BYTES)) {
        std::cerr << "[tx] write failed — closing port\n";
        g_port.close();
        return false;
    }
    return true;
}

void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [--port <name>] [--id <1..62>]\n"
        << "  --port    Serial port (Linux: /dev/ttyACM0, Windows: COM5).\n"
        << "            If omitted, auto-discovers by USB VID/PID.\n"
        << "  --id      SPARK MAX CAN device ID (default 1).\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string port_name;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--port" || a == "-p") && i + 1 < argc) {
            port_name = argv[++i];
        } else if ((a == "--id" || a == "-i") && i + 1 < argc) {
            g_device_id = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (a == "--help" || a == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << a << "\n";
            print_usage(argv[0]);
            return 2;
        }
    }

    if (g_device_id < 1 || g_device_id > 62) {
        std::cerr << "Device ID must be in 1..62\n";
        return 2;
    }

    if (port_name.empty()) {
        auto found = SerialPort::list_sparkmax_ports();
        if (found.empty()) {
            std::cerr << "No SPARK MAX detected by USB VID/PID. "
                         "Plug it in or pass --port explicitly.\n";
            return 1;
        }
        port_name = found.front();
        std::cout << "Auto-detected SPARK MAX on " << port_name << "\n";
        if (found.size() > 1) {
            std::cout << "(" << found.size()
                      << " controllers present — using the first; "
                         "use --port to pick another)\n";
        }
    }

    {
        std::lock_guard<std::mutex> lk(g_port_mutex);
        if (!g_port.open(port_name)) {
            std::cerr << "Failed to open " << port_name << "\n";
            return 1;
        }
        Packet hb = Packet::heartbeat();
        g_port.write_all(hb.bytes, PACKET_BYTES);  // wake the controller
    }
    std::cout << "Opened " << port_name
              << " (device_id=" << g_device_id << ")\n";

    std::thread hb_thread(heartbeat_thread);

    std::cout << "\nEnter position setpoint in rotations ("
              << MIN_SETPOINT << ".." << MAX_SETPOINT
              << "), 'q' to quit:\n";

    std::string line;
    while (true) {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line)) break;

        if (line.empty()) continue;
        if (line == "q" || line == "quit" || line == "exit") break;

        char* end = nullptr;
        const float v = std::strtof(line.c_str(), &end);
        if (end == line.c_str()) {
            std::cout << "  not a number\n";
            continue;
        }
        if (v < MIN_SETPOINT || v > MAX_SETPOINT) {
            std::cout << "  out of range (" << MIN_SETPOINT
                      << ".." << MAX_SETPOINT << ")\n";
            continue;
        }
        if (send_setpoint(v)) {
            std::cout << "  setpoint -> " << v << " rot\n";
        }
    }

    g_stop.store(true);
    if (hb_thread.joinable()) hb_thread.join();
    {
        std::lock_guard<std::mutex> lk(g_port_mutex);
        g_port.close();
    }
    std::cout << "Closed.\n";
    return 0;
}
