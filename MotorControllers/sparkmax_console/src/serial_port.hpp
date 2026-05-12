#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace spark {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& port_name);
    void close();
    bool is_open() const;
    bool write_all(const uint8_t* data, size_t len);

    // Auto-discovery: returns paths/names of attached SPARK MAX controllers.
    // Linux: e.g. {"/dev/ttyACM0"}. Windows: e.g. {"COM5"}.
    static std::vector<std::string> list_sparkmax_ports();

private:
#ifdef _WIN32
    void* handle_ = nullptr;     // HANDLE
#else
    int   fd_    = -1;
#endif
};

}  // namespace spark
