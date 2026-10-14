#include "ublox_i2c_core/ublox_i2c_handler.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <dirent.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <cstring>

// Walk /sys/class/tty/ttyACM* -> resolve the backing USB device -> walk up
// its sysfs parents looking for a "serial" attribute matching target_serial.
// This is the same lookup verified manually during hardware provisioning
// (via /sys/bus/usb/devices/*/serial) to distinguish the two otherwise
// identical X20P modules.
std::string find_tty_by_usb_serial(const std::string& target_serial) {
    DIR* d = opendir("/sys/class/tty");
    if (!d) return "";

    std::string found;
    struct dirent* entry;
    while ((entry = readdir(d)) != nullptr) {
        std::string name = entry->d_name;
        if (name.rfind("ttyACM", 0) != 0) continue;

        // realpath() follows the "device" symlink AND resolves the resulting
        // "../../../1-3.4:1.0"-style relative target in one step -- doing this
        // manually (readlink() then string-concatenating onto the symlink's
        // own path rather than its dirname) silently drops a path component.
        std::string device_link = "/sys/class/tty/" + name + "/device";
        char abs_path[4096];
        if (!realpath(device_link.c_str(), abs_path)) continue;

        std::string path(abs_path);
        // Walk up a handful of parent directories (interface -> USB device)
        // looking for the "serial" sysfs attribute.
        for (int i = 0; i < 6 && !path.empty(); i++) {
            std::ifstream f(path + "/serial");
            if (f.good()) {
                std::string s;
                std::getline(f, s);
                if (s == target_serial) {
                    found = "/dev/" + name;
                }
                break;  // this level has a "serial" file either way; stop walking up
            }
            size_t pos = path.find_last_of('/');
            if (pos == std::string::npos) break;
            path = path.substr(0, pos);
        }
        if (!found.empty()) break;
    }
    closedir(d);
    return found;
}

static bool configure_raw_serial(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return false;
    cfmakeraw(&tty);
    // USB CDC-ACM ignores the actual baud setting, but set one anyway for
    // any userspace tooling that inspects it.
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

UbloxI2C::UbloxI2C(const std::string& usb_serial_string, const std::string& fallback_device) {
    device_path_ = find_tty_by_usb_serial(usb_serial_string);
    if (device_path_.empty()) {
        device_path_ = fallback_device;
    }
    if (device_path_.empty()) {
        fd_ = -1;
        return;
    }
    fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ >= 0) {
        configure_raw_serial(fd_);
    }
}

UbloxI2C::~UbloxI2C() {
    if (fd_ >= 0) close(fd_);
}

std::vector<uint8_t> UbloxI2C::read_bus() {
    if (fd_ < 0) return {};
    uint8_t buf[1024];
    int r = read(fd_, buf, sizeof(buf));
    if (r > 0) {
        return std::vector<uint8_t>(buf, buf + r);
    }
    return {};
}

void UbloxI2C::write_rtcm(const std::vector<uint8_t>& data) {
    if (fd_ >= 0) {
        write(fd_, data.data(), data.size());
    }
}

void UbloxI2C::write_ubx(const std::vector<uint8_t>& data) {
    if (fd_ >= 0) {
        write(fd_, data.data(), data.size());
    }
}
