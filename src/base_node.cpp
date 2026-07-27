#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pvt.hpp"
#include "ublox_i2c_core/ublox_i2c_handler.hpp"
#include <unistd.h>
#include <vector>
#include <cstring>

template<typename T>
static T buf_read(const std::vector<uint8_t>& buf, size_t offset) {
    T val; std::memcpy(&val, &buf[offset], sizeof(T)); return val;
}

class BaseNode : public rclcpp::Node {
public:
    BaseNode() : Node("base_node") {
        fix_pub_  = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        info_pub_ = this->create_publisher<std_msgs::msg::String>("/gps/info", 10);
        pvt_pub_  = this->create_publisher<ublox_ubx_msgs::msg::UBXNavPVT>("/gps/ubx_nav_pvt", 10);

        i2c_ = std::make_unique<UbloxI2C>(7, 0x42);

        // When false (default), behave exactly like the original: isotropic hAcc/vAcc
        // covariance, every PVT published, no NAV-COV. Set true to enable the receiver's
        // true NAV-COV matrix + duplicate-covariance guard for the ZED SDK.
        use_nav_cov_ = this->declare_parameter<bool>("use_nav_cov", false);

        send_ubx_set_uart2_baud(460800);
        send_ubx_cfg_rate(40, 5);
        send_ubx_enable_uart2_ubx_out();
        send_ubx_enable_uart2_rtcm3_out();
        send_ubx_try_enable_4072_0_uart2();
        send_ubx_disable_rtcm3_input();
        if (use_nav_cov_) send_ubx_enable_nav_cov();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&BaseNode::loop, this));

        RCLCPP_INFO(get_logger(), "Base node ready. CFG-RATE meas=40ms nav=5. UART2 UBX+RTCM3 out enabled, RTCM3 input disabled. NAV-COV %s. Publishing /gps/fix + /gps/ubx_nav_pvt.",
            use_nav_cov_ ? "enabled" : "disabled");
    }

private:
    // UBX-CFG-VALSET (0x06 0x8A): set measurement + navigation rate via CFG-RATE-MEAS
    // (0x30210001, U2) and CFG-RATE-NAV (0x30210002, U2). Legacy UBX-CFG-RATE (0x06 0x08)
    // is confirmed UBX-ACK-NAK'd on this receiver (this X20P platform only implements
    // CFG-CFG/CFG-OTP/CFG-RST under class 0x06); keys verified against the ZED-X20P
    // Interface Description (UBXDOC-304424225-19888), section 6.9.17.
    void send_ubx_cfg_rate(uint16_t meas_ms, uint16_t nav_rate) {
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x8A, 0x10, 0x00};
        msg.push_back(0x00);        // version
        msg.push_back(0x01);        // layer: RAM-only
        msg.push_back(0x00); msg.push_back(0x00);  // reserved
        // key 0x30210001 = CFG-RATE-MEAS (U2), little-endian
        msg.push_back(0x01); msg.push_back(0x00); msg.push_back(0x21); msg.push_back(0x30);
        msg.push_back(meas_ms & 0xFF); msg.push_back((meas_ms >> 8) & 0xFF);
        // key 0x30210002 = CFG-RATE-NAV (U2), little-endian
        msg.push_back(0x02); msg.push_back(0x00); msg.push_back(0x21); msg.push_back(0x30);
        msg.push_back(nav_rate & 0xFF); msg.push_back((nav_rate >> 8) & 0xFF);
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        i2c_->write_ubx(msg);
        usleep(150000);
    }

    // UBX-CFG-VALSET (0x06 0x8A): explicitly set CFG-UART2-BAUDRATE (0x40530001, U4).
    // Default on this receiver is 38400 per the ZED-X20P Interface Description, and this
    // node never talks over UART2 itself (I2C only) so changing it here is safe. Pins both
    // ends of the base-to-rover UART2 link to a known, matching value — a mismatch here
    // would explain why only short/simple RTCM messages ever survive on the rover side while
    // longer MSM/4072.0 frames never do (short frames have much better odds of passing
    // checksum by chance under bit corruption from a baud mismatch).
    void send_ubx_set_uart2_baud(uint32_t baud) {
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00};
        msg.push_back(0x00);        // version
        msg.push_back(0x01);        // layer: RAM-only
        msg.push_back(0x00); msg.push_back(0x00);  // reserved
        // key 0x40530001, little-endian
        msg.push_back(0x01); msg.push_back(0x00); msg.push_back(0x53); msg.push_back(0x40);
        msg.push_back(baud & 0xFF); msg.push_back((baud >> 8) & 0xFF);
        msg.push_back((baud >> 16) & 0xFF); msg.push_back((baud >> 24) & 0xFF);
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        i2c_->write_ubx(msg);
        usleep(150000);
    }

    // UBX-CFG-VALSET (0x06 0x8A): enable UBX as an output protocol on UART2 (port outProtoMask).
    // key 0x10760001 = CFG-UART2OUTPROT-UBX, value 1. Verified against ZED-F9P Interface
    // Description (UBX-18010854): default is 0 (false); RTCM3X out on UART2 defaults to 1
    // and is left untouched by this call.
    void send_ubx_enable_uart2_ubx_out() {
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00};
        msg.push_back(0x00);        // version
        msg.push_back(0x01);        // layer: 0x01 = RAM-only (testing); 0x07 = RAM+BBR+Flash (persist)
        msg.push_back(0x00); msg.push_back(0x00);  // reserved
        // key 0x10760001, little-endian
        msg.push_back(0x01); msg.push_back(0x00); msg.push_back(0x76); msg.push_back(0x10);
        msg.push_back(0x01);        // value (L) = 1
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        i2c_->write_ubx(msg);
        usleep(150000);
    }

    // UBX-CFG-VALSET (0x06 0x8A): enable RTCM3X output protocol + the moving-base RTCM3
    // message set (1005, MSM4+MSM7, 1230) on UART2, at rate 1 (every epoch).
    // Keys verified against the ZED-X20P Interface Description (u-blox-20-HPG-2.00,
    // UBXDOC-304424225-19888) — this receiver is a ZED-X20P, not a ZED-F9P.
    void send_ubx_enable_uart2_rtcm3_out() {
        static const std::pair<uint32_t, uint8_t> kItems[] = {
            {0x10760004, 1},  // CFG-UART2OUTPROT-RTCM3X
            {0x209102bf, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1005_UART2
            {0x20910360, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1074_UART2 (GPS MSM4)
            {0x209102ce, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1077_UART2 (GPS MSM7)
            {0x20910365, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1084_UART2 (GLONASS MSM4)
            {0x209102d3, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1087_UART2 (GLONASS MSM7)
            {0x2091036a, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1094_UART2 (Galileo MSM4)
            {0x2091031a, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1097_UART2 (Galileo MSM7)
            {0x2091036f, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1124_UART2 (BeiDou MSM4)
            {0x209102d8, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1127_UART2 (BeiDou MSM7)
            {0x20910305, 1},  // CFG-MSGOUT-RTCM_3X_TYPE1230_UART2
        };
        uint16_t payload_len = 4 + sizeof(kItems) / sizeof(kItems[0]) * 5;
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x8A,
            (uint8_t)(payload_len & 0xFF), (uint8_t)((payload_len >> 8) & 0xFF)};
        msg.push_back(0x00);        // version
        msg.push_back(0x01);        // layer: 0x01 = RAM-only (testing); 0x07 = RAM+BBR+Flash (persist)
        msg.push_back(0x00); msg.push_back(0x00);  // reserved
        for (const auto& item : kItems) {
            uint32_t key = item.first;
            msg.push_back(key & 0xFF); msg.push_back((key >> 8) & 0xFF);
            msg.push_back((key >> 16) & 0xFF); msg.push_back((key >> 24) & 0xFF);
            msg.push_back(item.second);
        }
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        i2c_->write_ubx(msg);
        usleep(150000);
    }

    // UBX-CFG-VALSET (0x06 0x8A): PROBE — try CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART2 = 0x20910300.
    // This key is undocumented in both the F9P and X20P interface descriptions (RTCM-3X-TYPE4072_0
    // only appears there as the RTCM-native Class/ID 0xf5 0xfe, which is meaningless for VALSET
    // and belongs to the now-confirmed-dead legacy CFG-MSG path on this receiver). 0x20910300 is
    // one past CFG-MSGOUT-RTCM_3X_TYPE1097_UART2's neighbors and sits in a key range this X20P doc
    // otherwise leaves completely unclaimed, and every other CFG-MSGOUT-RTCM_3X_TYPE*_UART2 key
    // verified identical between F9P and X20P — so it's worth trying on its own. Sent as its own
    // standalone VALSET (not merged into send_ubx_enable_uart2_rtcm3_out()'s batch) so that if the
    // receiver NAKs an unrecognized key, it can't take the already-confirmed-working keys down
    // with it. Watch the UBX-ACK-ACK/NAK log for class 0x06 id 0x8A right after this call.
    void send_ubx_try_enable_4072_0_uart2() {
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00};
        msg.push_back(0x00);        // version
        msg.push_back(0x01);        // layer: RAM-only
        msg.push_back(0x00); msg.push_back(0x00);  // reserved
        // key 0x20910300, little-endian
        msg.push_back(0x00); msg.push_back(0x03); msg.push_back(0x91); msg.push_back(0x20);
        msg.push_back(0x01);        // value (U1) = 1
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        i2c_->write_ubx(msg);
        usleep(150000);
    }

    // UBX-CFG-VALSET (0x06 0x8A): disable RTCM3X as an INPUT protocol on UART2 and I2C.
    // Both default to enabled (1/true) on this receiver per the ZED-X20P Interface
    // Description (UBXDOC-304424225-19888, CFG-UART2INPROT-RTCM3X and CFG-I2CINPROT-RTCM3X
    // config defaults tables). The base is meant to be a one-way corrections source; if
    // anything reaches its UART2 RX (e.g. symmetric wiring reflecting the rover's own
    // output back) or its I2C bus, a receiver that still accepts RTCM3 input will apply it
    // and start computing an RTK solution against it — which is exactly why the base was
    // reporting its own fix as RTK FLOAT instead of a plain autonomous/standalone fix.
    void send_ubx_disable_rtcm3_input() {
        static const std::pair<uint32_t, uint8_t> kItems[] = {
            {0x10750004, 0},  // CFG-UART2INPROT-RTCM3X = 0
            {0x10710004, 0},  // CFG-I2CINPROT-RTCM3X = 0
        };
        uint16_t payload_len = 4 + sizeof(kItems) / sizeof(kItems[0]) * 5;
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x8A,
            (uint8_t)(payload_len & 0xFF), (uint8_t)((payload_len >> 8) & 0xFF)};
        msg.push_back(0x00);        // version
        msg.push_back(0x01);        // layer: RAM-only
        msg.push_back(0x00); msg.push_back(0x00);  // reserved
        for (const auto& item : kItems) {
            uint32_t key = item.first;
            msg.push_back(key & 0xFF); msg.push_back((key >> 8) & 0xFF);
            msg.push_back((key >> 16) & 0xFF); msg.push_back((key >> 24) & 0xFF);
            msg.push_back(item.second);
        }
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        i2c_->write_ubx(msg);
        usleep(150000);
    }

    // UBX-CFG-VALSET (0x06 0x8A): enable UBX-NAV-COV output on I2C every nav epoch.
    // key 0x20910087 = CFG-MSGOUT-UBX_NAV_COV_I2C, value 1.
    // NOTE: verify this key ID against UBX-MON-VER protocol version before trusting.
    void send_ubx_enable_nav_cov() {
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00};
        msg.push_back(0x00);        // version
        msg.push_back(0x01);        // layer: 0x01 = RAM-only (testing); 0x07 = RAM+BBR+Flash (persist)
        msg.push_back(0x00); msg.push_back(0x00);  // reserved
        // key 0x20910087, little-endian
        msg.push_back(0x87); msg.push_back(0x00); msg.push_back(0x91); msg.push_back(0x20);
        msg.push_back(0x01);        // value (U1) = 1
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        i2c_->write_ubx(msg);
        usleep(150000);
    }

    bool verify_ubx_checksum(size_t start, uint16_t len) {
        if (start + len + 8 > buffer_.size()) return false;
        uint8_t cka = 0, ckb = 0;
        for (size_t k = 2; k < (size_t)(len + 6); k++) { cka += buffer_[start + k]; ckb += cka; }
        return (cka == buffer_[start + len + 6] && ckb == buffer_[start + len + 7]);
    }

    void loop() {
        while (true) {
            auto data = i2c_->read_bus();
            if (data.empty()) break;
            buffer_.insert(buffer_.end(), data.begin(), data.end());
            if (buffer_.size() > 65536) break;
        }

        size_t i = 0;
        while (i + 8 <= buffer_.size()) {
            if (buffer_[i] == 0xB5 && buffer_[i+1] == 0x62) {
                uint8_t  cls = buffer_[i+2];
                uint8_t  id  = buffer_[i+3];
                uint16_t len = buffer_[i+4] | (buffer_[i+5] << 8);
                if (len > 1024) { i++; continue; }
                size_t total = 8 + len;
                if (i + total <= buffer_.size()) {
                    if (verify_ubx_checksum(i, len)) {
                        if      (cls == 0x01 && id == 0x07) handle_pvt(i);
                        else if (cls == 0x01 && id == 0x36 && use_nav_cov_) handle_nav_cov(i);
                        i += total; continue;
                    }
                } else break;
            } else if (buffer_[i] == 0xD3 && i + 2 < buffer_.size()) {
                uint16_t rtcm_len = ((buffer_[i+1] & 0x03) << 8) | buffer_[i+2];
                if (rtcm_len > 1024) { i++; continue; }
                size_t total_rtcm = rtcm_len + 6;
                if (i + total_rtcm <= buffer_.size()) { i += total_rtcm; continue; }
                else break;
            }
            i++;
        }
        if (i > 0) buffer_.erase(buffer_.begin(), buffer_.begin() + i);
        if (buffer_.size() > 16384) buffer_.clear();
    }

    // UBX-NAV-COV (0x01 0x36): full 3x3 position covariance in NED frame, m^2.
    // Cached and matched to a PVT of the same nav epoch by iTOW (NAV-COV carries no position).
    void handle_nav_cov(size_t idx) {
        const size_t B = idx + 6;

        uint32_t iTOW        = buf_read<uint32_t>(buffer_, B + 0);
        uint8_t  posCovValid = buffer_[B + 5];
        if (!posCovValid) return;

        float NN = buf_read<float>(buffer_, B + 16);
        float NE = buf_read<float>(buffer_, B + 20);
        float ND = buf_read<float>(buffer_, B + 24);
        float EE = buf_read<float>(buffer_, B + 28);
        float ED = buf_read<float>(buffer_, B + 32);
        float DD = buf_read<float>(buffer_, B + 36);

        // NED -> ENU signed-permutation remap (row-major 3x3: [EE EN EU; NE NN NU; UE UN UU])
        nav_cov_enu_[0] = EE;       nav_cov_enu_[1] = NE;       nav_cov_enu_[2] = -ED;
        nav_cov_enu_[3] = NE;       nav_cov_enu_[4] = NN;       nav_cov_enu_[5] = -ND;
        nav_cov_enu_[6] = -ED;      nav_cov_enu_[7] = -ND;      nav_cov_enu_[8] = DD;

        nav_cov_itow_  = iTOW;
        nav_cov_valid_ = true;
    }

    void handle_pvt(size_t idx) {
        const size_t B = idx + 6;

        uint32_t iTOW    = buf_read<uint32_t>(buffer_, B +  0);
        uint16_t year    = buf_read<uint16_t>(buffer_, B +  4);
        uint8_t  month   = buffer_[B +  6];
        uint8_t  day     = buffer_[B +  7];
        uint8_t  hour    = buffer_[B +  8];
        uint8_t  min_    = buffer_[B +  9];
        uint8_t  sec     = buffer_[B + 10];
        uint8_t  valid   = buffer_[B + 11];
        uint32_t tAcc    = buf_read<uint32_t>(buffer_, B + 12);
        int32_t  nano    = buf_read<int32_t> (buffer_, B + 16);
        uint8_t  fixType = buffer_[B + 20];
        uint8_t  flags   = buffer_[B + 21];
        uint8_t  flags2  = buffer_[B + 22];
        uint8_t  numSV   = buffer_[B + 23];

        // flags breakdown
        bool    gnssFixOk    = (flags & 0x01) != 0;
        bool    diffSoln     = (flags & 0x02) != 0;
        uint8_t psmState     = (flags >> 2) & 0x07;
        bool    headVehValid = (flags & 0x20) != 0;
        uint8_t carrSolnRaw  = (flags >> 6) & 0x03;

        // flags2 breakdown
        bool confirmedAvail = (flags2 & 0x20) != 0;
        bool confirmedDate  = (flags2 & 0x40) != 0;
        bool confirmedTime  = (flags2 & 0x80) != 0;

        // valid breakdown
        bool validDate     = (valid & 0x01) != 0;
        bool validTime     = (valid & 0x02) != 0;
        bool fullyResolved = (valid & 0x04) != 0;
        bool validMag      = (valid & 0x08) != 0;

        int32_t  lon_raw  = buf_read<int32_t> (buffer_, B + 24);
        int32_t  lat_raw  = buf_read<int32_t> (buffer_, B + 28);
        int32_t  height   = buf_read<int32_t> (buffer_, B + 32);
        int32_t  hMSL     = buf_read<int32_t> (buffer_, B + 36);
        uint32_t hAcc_raw = buf_read<uint32_t>(buffer_, B + 40);
        uint32_t vAcc_raw = buf_read<uint32_t>(buffer_, B + 44);
        int32_t  velN     = buf_read<int32_t> (buffer_, B + 48);
        int32_t  velE     = buf_read<int32_t> (buffer_, B + 52);
        int32_t  velD     = buf_read<int32_t> (buffer_, B + 56);
        int32_t  gSpeed   = buf_read<int32_t> (buffer_, B + 60);
        int32_t  headMot  = buf_read<int32_t> (buffer_, B + 64);
        uint32_t sAcc     = buf_read<uint32_t>(buffer_, B + 68);
        uint32_t headAcc  = buf_read<uint32_t>(buffer_, B + 72);
        uint16_t pDOP     = buf_read<uint16_t>(buffer_, B + 76);
        int32_t  headVeh  = buf_read<int32_t> (buffer_, B + 84);

        float hAcc_m = hAcc_raw / 1000.0f;
        float vAcc_m = vAcc_raw / 1000.0f;

        // NavSatFix
        auto fix_msg = sensor_msgs::msg::NavSatFix();
        fix_msg.header.stamp    = this->now();
        fix_msg.header.frame_id = "base_link";
        fix_msg.latitude   = lat_raw * 1e-7;
        fix_msg.longitude  = lon_raw * 1e-7;
        fix_msg.altitude   = hMSL / 1000.0;
        if (use_nav_cov_ && nav_cov_valid_ && nav_cov_itow_ == iTOW) {
            // Use the receiver's true ENU covariance from a same-epoch NAV-COV.
            for (size_t k = 0; k < 9; k++) fix_msg.position_covariance[k] = nav_cov_enu_[k];
            fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
        } else {
            // Original isotropic hAcc/vAcc approximation.
            fix_msg.position_covariance[0] = hAcc_m * hAcc_m;
            fix_msg.position_covariance[4] = hAcc_m * hAcc_m;
            fix_msg.position_covariance[8] = vAcc_m * vAcc_m;
            fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        }
        if      (carrSolnRaw == 2) fix_msg.status.status = 2;
        else if (carrSolnRaw == 1) fix_msg.status.status = 1;
        else if (fixType     >= 3) fix_msg.status.status = 0;
        else                       fix_msg.status.status = -1;

        if (use_nav_cov_) {
            // Duplicate-covariance guard: the ZED SDK rejects repeated covariance values
            // ("GNSS DATA COVARIANCE MUST VARY"), so skip publishing a bit-identical matrix.
            bool cov_identical = true;
            for (size_t k = 0; k < 9; k++) {
                if (fix_msg.position_covariance[k] != last_pub_cov_[k]) { cov_identical = false; break; }
            }
            if (cov_identical) {
                RCLCPP_DEBUG(get_logger(), "Skipping /gps/fix publish: covariance identical to previous epoch.");
                // skip publish
            } else {
                for (size_t k = 0; k < 9; k++) last_pub_cov_[k] = fix_msg.position_covariance[k];
                fix_pub_->publish(fix_msg);
            }
        } else {
            // Original behavior: always publish.
            fix_pub_->publish(fix_msg);
        }

        // UBXNavPVT — exact field names from message definition
        auto pvt = ublox_ubx_msgs::msg::UBXNavPVT();
        pvt.header.stamp    = this->now();
        pvt.header.frame_id = "base_link";

        pvt.itow           = iTOW;
        pvt.year           = year;
        pvt.month          = month;
        pvt.day            = day;
        pvt.hour           = hour;
        pvt.min            = min_;
        pvt.sec            = sec;
        pvt.valid_date     = validDate;
        pvt.valid_time     = validTime;
        pvt.fully_resolved = fullyResolved;
        pvt.valid_mag      = validMag;
        pvt.t_acc          = tAcc;
        pvt.nano           = nano;

        pvt.gps_fix.fix_type = fixType;
        pvt.gnss_fix_ok      = gnssFixOk;
        pvt.diff_soln        = diffSoln;
        pvt.psm.state        = psmState;
        pvt.head_veh_valid   = headVehValid;
        pvt.carr_soln.status = carrSolnRaw;
        pvt.confirmed_avail  = confirmedAvail;
        pvt.confirmed_date   = confirmedDate;
        pvt.confirmed_time   = confirmedTime;

        pvt.num_sv   = numSV;
        pvt.lon      = lon_raw;
        pvt.lat      = lat_raw;
        pvt.height   = height;
        pvt.hmsl     = hMSL;
        pvt.h_acc    = hAcc_raw;
        pvt.v_acc    = vAcc_raw;
        pvt.vel_n    = velN;
        pvt.vel_e    = velE;
        pvt.vel_d    = velD;
        pvt.g_speed  = gSpeed;
        pvt.head_mot = headMot;
        pvt.s_acc    = sAcc;
        pvt.head_acc = headAcc;
        pvt.p_dop    = pDOP;
        pvt.head_veh = headVeh;

        pvt_pub_->publish(pvt);

        // Info string
        char buf[160];
        if      (carrSolnRaw == 2) snprintf(buf, sizeof(buf), "BASE RTK FIXED | Acc: %.4fm | Sats: %u | Speed: %.3f m/s", hAcc_m, numSV, gSpeed / 1000.0f);
        else if (carrSolnRaw == 1) snprintf(buf, sizeof(buf), "BASE RTK FLOAT | Acc: %.4fm | Sats: %u | Speed: %.3f m/s", hAcc_m, numSV, gSpeed / 1000.0f);
        else if (fixType     >= 3) snprintf(buf, sizeof(buf), "BASE 3D FIX    | Acc: %.4fm | Sats: %u | Speed: %.3f m/s", hAcc_m, numSV, gSpeed / 1000.0f);
        else                       snprintf(buf, sizeof(buf), "BASE NO FIX    | Acc: %.4fm | Sats: %u", hAcc_m, numSV);
        auto info_msg = std_msgs::msg::String();
        info_msg.data = buf;
        info_pub_->publish(info_msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "%s | Fix:%u CarrSoln:%u HeadVeh:%.2fdeg", buf, fixType, carrSolnRaw, headVeh * 1e-5);
    }

    std::vector<uint8_t> buffer_;
    std::unique_ptr<UbloxI2C> i2c_;

    bool use_nav_cov_ = false;  // parameter: gate all NAV-COV behavior (default = original)

    // NAV-COV cache (ENU, m^2), matched to PVT by iTOW.
    double   nav_cov_enu_[9] = {0};
    uint32_t nav_cov_itow_   = 0;
    bool     nav_cov_valid_  = false;
    // Last published covariance, for the duplicate guard (-1 forces first publish).
    double   last_pub_cov_[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr     fix_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           info_pub_;
    rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr pvt_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseNode>());
    rclcpp::shutdown();
    return 0;
}
