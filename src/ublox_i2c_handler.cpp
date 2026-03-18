#include "ublox_i2c_core/ublox_i2c_handler.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <vector>
#include <string>

UbloxI2C::UbloxI2C(int bus, int addr) {
    std::string dev = "/dev/i2c-" + std::to_string(bus);
    fd_ = open(dev.c_str(), O_RDWR);
    if (fd_ >= 0) {
        ioctl(fd_, I2C_SLAVE, addr);
    }
}

std::vector<uint8_t> UbloxI2C::read_bus() {
    uint8_t buf[1024];
    int r = read(fd_, buf, sizeof(buf));
    if (r > 0) return std::vector<uint8_t>(buf, buf + r);
    return {};
}

void UbloxI2C::write_rtcm(const std::vector<uint8_t>& data) {
    if (fd_ >= 0) {
        write(fd_, data.data(), data.size());
    }
}
