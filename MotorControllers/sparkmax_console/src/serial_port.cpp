#include "serial_port.hpp"
#include "spark_protocol.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
  #include <setupapi.h>
  #include <devguid.h>
  #include <cfgmgr32.h>
  #pragma comment(lib, "setupapi.lib")
#else
  #include <errno.h>
  #include <fcntl.h>
  #include <termios.h>
  #include <unistd.h>
  #include <dirent.h>
  #include <sys/stat.h>
  #include <fstream>
#endif

namespace spark {

// ---------------------------------------------------------------------------
// Linux implementation
// ---------------------------------------------------------------------------
#ifndef _WIN32

bool SerialPort::open(const std::string& port_name) {
    close();
    fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_EXCL);
    if (fd_ < 0) return false;

    struct termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) { close(); return false; }

    ::cfsetispeed(&tty, B115200);
    ::cfsetospeed(&tty, B115200);

    tty.c_cflag |=  (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;  tty.c_cflag |= CS8;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_lflag  =  0;
    tty.c_oflag  =  0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SerialPort::is_open() const { return fd_ >= 0; }

bool SerialPort::write_all(const uint8_t* data, size_t len) {
    if (fd_ < 0) return false;
    size_t written = 0;
    while (written < len) {
        ssize_t n = ::write(fd_, data + written, len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        written += static_cast<size_t>(n);
    }
    return true;
}

static std::string read_sysfs_line(const std::string& path) {
    std::ifstream f(path);
    std::string s;
    if (f) std::getline(f, s);
    return s;
}

static bool sysfs_matches_sparkmax(const std::string& tty_basename) {
    const std::string sys = "/sys/class/tty/" + tty_basename + "/device/";
    try {
        unsigned vid = std::stoul(read_sysfs_line(sys + "../idVendor"),  nullptr, 16);
        unsigned pid = std::stoul(read_sysfs_line(sys + "../idProduct"), nullptr, 16);
        return vid == SPARK_USB_VID && pid == SPARK_USB_PID;
    } catch (...) {
        return false;
    }
}

std::vector<std::string> SerialPort::list_sparkmax_ports() {
    std::vector<std::string> result;
    DIR* d = ::opendir("/dev");
    if (!d) return result;

    struct dirent* e;
    while ((e = ::readdir(d)) != nullptr) {
        std::string name = e->d_name;
        if (name.rfind("ttyACM", 0) != 0) continue;
        if (sysfs_matches_sparkmax(name)) {
            result.push_back("/dev/" + name);
        }
    }
    ::closedir(d);
    std::sort(result.begin(), result.end());
    return result;
}

// ---------------------------------------------------------------------------
// Windows implementation
// ---------------------------------------------------------------------------
#else

bool SerialPort::open(const std::string& port_name) {
    close();

    // Use \\.\COMx form to support COM10+
    std::string path = "\\\\.\\" + port_name;

    HANDLE h = ::CreateFileA(
        path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,                  // exclusive
        nullptr,
        OPEN_EXISTING,
        0,
        nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!::GetCommState(h, &dcb)) { ::CloseHandle(h); return false; }

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow    = FALSE;
    dcb.fOutxDsrFlow    = FALSE;
    dcb.fDtrControl     = DTR_CONTROL_ENABLE;
    dcb.fRtsControl     = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX  = FALSE;

    if (!::SetCommState(h, &dcb)) { ::CloseHandle(h); return false; }

    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout         = MAXDWORD;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.ReadTotalTimeoutConstant    = 0;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = 100;
    if (!::SetCommTimeouts(h, &to)) { ::CloseHandle(h); return false; }

    handle_ = h;
    return true;
}

void SerialPort::close() {
    if (handle_) { ::CloseHandle(static_cast<HANDLE>(handle_)); handle_ = nullptr; }
}

bool SerialPort::is_open() const { return handle_ != nullptr; }

bool SerialPort::write_all(const uint8_t* data, size_t len) {
    if (!handle_) return false;
    DWORD written = 0;
    if (!::WriteFile(static_cast<HANDLE>(handle_), data,
                     static_cast<DWORD>(len), &written, nullptr)) {
        return false;
    }
    return written == len;
}

std::vector<std::string> SerialPort::list_sparkmax_ports() {
    std::vector<std::string> result;

    HDEVINFO devs = ::SetupDiGetClassDevsA(
        &GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (devs == INVALID_HANDLE_VALUE) return result;

    char vidpid_needle[32];
    std::snprintf(vidpid_needle, sizeof(vidpid_needle),
                  "VID_%04X&PID_%04X", SPARK_USB_VID, SPARK_USB_PID);

    SP_DEVINFO_DATA info{};
    info.cbSize = sizeof(info);

    for (DWORD i = 0; ::SetupDiEnumDeviceInfo(devs, i, &info); ++i) {
        char hwid[512] = {};
        DWORD type = 0;
        if (!::SetupDiGetDeviceRegistryPropertyA(
                devs, &info, SPDRP_HARDWAREID, &type,
                reinterpret_cast<PBYTE>(hwid), sizeof(hwid), nullptr)) {
            continue;
        }

        // Case-insensitive search for VID_xxxx&PID_xxxx
        std::string hw(hwid);
        std::transform(hw.begin(), hw.end(), hw.begin(), ::toupper);
        if (hw.find(vidpid_needle) == std::string::npos) continue;

        // Read PortName from device registry key
        HKEY key = ::SetupDiOpenDevRegKey(
            devs, &info, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (key == INVALID_HANDLE_VALUE) continue;

        char port_name[64] = {};
        DWORD sz = sizeof(port_name);
        DWORD reg_type = 0;
        LONG rc = ::RegQueryValueExA(key, "PortName", nullptr, &reg_type,
                                     reinterpret_cast<LPBYTE>(port_name), &sz);
        ::RegCloseKey(key);
        if (rc == ERROR_SUCCESS && port_name[0]) {
            result.emplace_back(port_name);
        }
    }

    ::SetupDiDestroyDeviceInfoList(devs);
    std::sort(result.begin(), result.end());
    return result;
}

#endif

}  // namespace spark
