#ifndef UBLOX_I2C_HANDLER_HPP
#define UBLOX_I2C_HANDLER_HPP

#include <vector>
#include <cstdint>
#include <string>

struct NavHpPosLlh {
    uint8_t version; uint8_t reserved1; uint16_t reserved2;
    uint32_t iTOW; int32_t lon; int32_t lat; int32_t height; int32_t hMSL;
    int8_t lonHp; int8_t latHp; int8_t heightHp; int8_t hMSLHp;
    uint32_t hAcc; uint32_t vAcc; uint32_t flags;
};

struct NavPvt {
    uint32_t iTOW; uint16_t year; uint8_t month, day, hour, min, sec;
    uint8_t valid; uint32_t tAcc; int32_t nano; uint8_t fixType;
    uint8_t flags; uint8_t flags2; uint8_t numSV;
    int32_t lon; int32_t lat; int32_t height; int32_t hMSL;
    uint32_t hAcc; uint32_t vAcc;
    int32_t velN; int32_t velE; int32_t velD; int32_t gSpeed;
    int32_t headMot; uint32_t sAcc; uint32_t headAcc;
    uint16_t pDOP; uint16_t reserved1; uint32_t reserved2;
};

struct NavRelPosNed {
    uint8_t version; uint8_t reserved1; uint16_t refStationId;
    uint32_t iTOW; int32_t relPosN; int32_t relPosE; int32_t relPosD;
    int32_t relPosHPN; int32_t relPosHPE; int32_t relPosHPD;
    uint32_t reserved2; uint32_t accN; uint32_t accE; uint32_t accD;
    uint32_t flags;
};

// Finds the /dev/ttyACM* node whose USB iSerialNumber matches usb_serial_string
// (e.g. "Tset" or "tseT"). Returns "" if no match is found. Shared by both
// UbloxI2C (below) and rover_node.cpp, which manages its own serial fd.
std::string find_tty_by_usb_serial(const std::string& usb_serial_string);

// NB: class name kept as UbloxI2C for compatibility with existing callers
// (base_node.cpp, i2c_manager_node.cpp) even though the transport is now the
// X20P's own USB CDC-ACM interface, not the I2C/DDC bus. The base and rover
// modules are physically identical and ship with no USB serial string, so
// they were each programmed with a distinct one (base="Tset", rover="tseT" --
// see supervisor/tools/program_ublox_serial.py); this class finds the
// /dev/ttyACM* node whose USB iSerialNumber matches the one requested.
class UbloxI2C {
public:
    // usb_serial_string: the programmed iSerialNumber to search for (e.g. "Tset").
    // fallback_device: used only if no USB device with that serial is found
    // (e.g. during bring-up before a module has been serial-programmed).
    explicit UbloxI2C(const std::string& usb_serial_string, const std::string& fallback_device = "");
    ~UbloxI2C();
    bool is_open() const { return fd_ >= 0; }
    std::string device_path() const { return device_path_; }
    std::vector<uint8_t> read_bus();
    void write_rtcm(const std::vector<uint8_t>& data);
    void write_ubx(const std::vector<uint8_t>& data);
private:
    int fd_ = -1;
    std::string device_path_;
};

#endif
