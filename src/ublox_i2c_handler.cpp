#include "ublox_i2c_core/ublox_i2c_handler.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <unistd.h>
#include <algorithm>
#include <vector>
#include <string>

UbloxI2C::UbloxI2C(int bus, int addr) {
    std::string dev = "/dev/i2c-" + std::to_string(bus);
    fd_ = open(dev.c_str(), O_RDWR);
    addr_ = static_cast<uint8_t>(addr);
    if (fd_ >= 0) {
        ioctl(fd_, I2C_SLAVE, addr);
    }
}

// Reads the DDC "bytes available" register pair (0xFD/0xFE) as a single combined
// I2C_RDWR transfer (repeated START, no STOP between the register-pointer write
// and the 2-byte read). Doing the write and read as separate read()/write() calls
// puts a STOP between them, which drops the device back onto the 0xFF streaming
// register — so the "count" read back is really just stream padding (0xFFFF),
// always > 1024, and read_bus() would never see "no data" (the original bug).
uint16_t UbloxI2C::bytes_available() {
    if (fd_ < 0) return 0;

    uint8_t reg = 0xFD;
    uint8_t avail[2] = {0, 0};

    struct i2c_msg msgs[2];
    msgs[0].addr  = addr_;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = addr_;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = 2;
    msgs[1].buf   = avail;

    struct i2c_rdwr_ioctl_data xfer;
    xfer.msgs  = msgs;
    xfer.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &xfer) < 0) return 0;
    return (static_cast<uint16_t>(avail[0]) << 8) | avail[1];
}

std::vector<uint8_t> UbloxI2C::read_bus() {
    uint16_t avail = bytes_available();
    if (avail == 0) return {};
    uint16_t to_read = std::min<uint16_t>(avail, 1024);
    std::vector<uint8_t> buf(to_read);
    int r = read(fd_, buf.data(), to_read);
    if (r > 0) {
        buf.resize(r);
        return buf;
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
