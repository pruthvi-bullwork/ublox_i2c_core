#ifndef UBLOX_I2C_HANDLER_HPP
#define UBLOX_I2C_HANDLER_HPP

#include <vector>
#include <cstdint>

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

class UbloxI2C {
public:
    UbloxI2C(int bus, int addr);
    std::vector<uint8_t> read_bus();
    void write_rtcm(const std::vector<uint8_t>& data);
private:
    int fd_;
};

#endif