// UBX-CFG-VALSET / UBX-CFG-VALGET / UBX-MON-VER frame builders.
//
// Reconstructed from the ublox-i2c-core 1.3.0 arm64 build (symbols ubx::*).
#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace ubx {

// CFG-VALSET layer bitmask (VALSET "layers" field).
constexpr uint8_t LAYER_RAM   = 0x01;
constexpr uint8_t LAYER_BBR   = 0x02;
constexpr uint8_t LAYER_FLASH = 0x04;

// CFG-VALGET layer selector (a single value, not a bitmask).
constexpr uint8_t VALGET_RAM     = 0;
constexpr uint8_t VALGET_BBR     = 1;
constexpr uint8_t VALGET_FLASH   = 2;
constexpr uint8_t VALGET_DEFAULT = 7;

// CFG-NAVSPG keys.
constexpr uint32_t CFG_NAVSPG_DYNMODEL      = 0x20110021;  // E1
constexpr uint32_t CFG_NAVSPG_INFIL_MINCNO  = 0x201100a3;  // U1 dBHz
constexpr uint32_t CFG_NAVSPG_INFIL_MINELEV = 0x201100a4;  // I1 deg
constexpr uint32_t CFG_NAVSPG_OUTFIL_PDOP   = 0x301100b1;  // U2 x0.1
constexpr uint32_t CFG_NAVSPG_OUTFIL_TDOP   = 0x301100b2;  // U2 x0.1
constexpr uint32_t CFG_NAVSPG_OUTFIL_PACC   = 0x301100b3;  // U2 m
constexpr uint32_t CFG_NAVSPG_OUTFIL_TACC   = 0x301100b4;  // U2 m

// CFG-MSGOUT keys.
constexpr uint32_t CFG_MSGOUT_UBX_NAV_SAT_I2C   = 0x20910015;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016;

// Value size in bytes, from the key's size field (bits 28..30).
inline size_t key_value_size(uint32_t key)
{
  switch ((key >> 28) & 0x7) {
    case 1:  // L (bit), stored in one byte
    case 2:  return 1;
    case 3:  return 2;
    case 4:  return 4;
    case 5:  return 8;
    default: return 0;
  }
}

// UBX-CFG-VALSET (0x06 0x8A), version 0, one or more key/value pairs.
inline std::vector<uint8_t> build_valset(
    const std::vector<std::pair<uint32_t, uint64_t>> &kv, uint8_t layers)
{
  std::vector<uint8_t> payload;
  payload.push_back(0x00);  // version
  payload.push_back(layers);
  payload.push_back(0x00);  // reserved
  payload.push_back(0x00);

  for (const auto &item : kv) {
    const uint32_t key = item.first;
    const size_t size = key_value_size(key);
    if (size == 0) continue;
    payload.push_back(key & 0xFF);
    payload.push_back((key >> 8) & 0xFF);
    payload.push_back((key >> 16) & 0xFF);
    payload.push_back((key >> 24) & 0xFF);
    for (size_t i = 0; i < size; ++i) {
      payload.push_back((item.second >> (8 * i)) & 0xFF);
    }
  }

  std::vector<uint8_t> frame = {
    0xB5, 0x62, 0x06, 0x8A,
    static_cast<uint8_t>(payload.size() & 0xFF),
    static_cast<uint8_t>((payload.size() >> 8) & 0xFF)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  uint8_t ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < frame.size(); ++i) {
    ck_a = ck_a + frame[i];
    ck_b = ck_b + ck_a;
  }
  frame.push_back(ck_a);
  frame.push_back(ck_b);
  return frame;
}

// UBX-CFG-VALGET (0x06 0x8B) poll for explicit keys from one layer.
inline std::vector<uint8_t> build_valget(
    const std::vector<uint32_t> &keys, uint8_t layer, uint16_t position = 0)
{
  std::vector<uint8_t> payload;
  payload.push_back(0x00);  // version (request)
  payload.push_back(layer);
  payload.push_back(position & 0xFF);
  payload.push_back((position >> 8) & 0xFF);

  for (uint32_t key : keys) {
    payload.push_back(key & 0xFF);
    payload.push_back((key >> 8) & 0xFF);
    payload.push_back((key >> 16) & 0xFF);
    payload.push_back((key >> 24) & 0xFF);
  }

  std::vector<uint8_t> frame = {
    0xB5, 0x62, 0x06, 0x8B,
    static_cast<uint8_t>(payload.size() & 0xFF),
    static_cast<uint8_t>((payload.size() >> 8) & 0xFF)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  uint8_t ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < frame.size(); ++i) {
    ck_a = ck_a + frame[i];
    ck_b = ck_b + ck_a;
  }
  frame.push_back(ck_a);
  frame.push_back(ck_b);
  return frame;
}

// UBX-MON-VER (0x0A 0x04) poll: empty payload.
inline std::vector<uint8_t> build_mon_ver_poll()
{
  std::vector<uint8_t> frame = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00};
  uint8_t ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < frame.size(); ++i) {
    ck_a = ck_a + frame[i];
    ck_b = ck_b + ck_a;
  }
  frame.push_back(ck_a);
  frame.push_back(ck_b);
  return frame;
}

inline const char *dyn_model_name(uint8_t m)
{
  switch (m) {
    case 0:  return "portable";
    case 2:  return "stationary";
    case 3:  return "pedestrian";
    case 4:  return "automotive";
    case 5:  return "sea";
    case 6:  return "airborne<1g";
    case 9:  return "wrist";
    case 11: return "mower";
    case 12: return "e-scooter";
    default: return "?";
  }
}

inline const char *key_name(uint32_t key)
{
  switch (key) {
    case CFG_NAVSPG_DYNMODEL:      return "CFG-NAVSPG-DYNMODEL";
    case CFG_NAVSPG_INFIL_MINCNO:  return "CFG-NAVSPG-INFIL_MINCNO";
    case CFG_NAVSPG_INFIL_MINELEV: return "CFG-NAVSPG-INFIL_MINELEV";
    case CFG_NAVSPG_OUTFIL_PDOP:   return "CFG-NAVSPG-OUTFIL_PDOP";
    case CFG_NAVSPG_OUTFIL_TDOP:   return "CFG-NAVSPG-OUTFIL_TDOP";
    case CFG_NAVSPG_OUTFIL_PACC:   return "CFG-NAVSPG-OUTFIL_PACC";
    case CFG_NAVSPG_OUTFIL_TACC:   return "CFG-NAVSPG-OUTFIL_TACC";
    default:                       return "(unnamed key)";
  }
}

// Name of a CFG-VALGET layer selector.
inline const char *layer_name(uint8_t layer)
{
  switch (layer) {
    case VALGET_RAM:     return "RAM";
    case VALGET_BBR:     return "BBR";
    case VALGET_FLASH:   return "Flash";
    case VALGET_DEFAULT: return "Default";
    default:             return "?";
  }
}

}  // namespace ubx
