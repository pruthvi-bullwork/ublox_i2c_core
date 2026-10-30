#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pvt.hpp"
#include "ublox_i2c_core/ublox_i2c_handler.hpp"
#include "ublox_i2c_core/ubx_valset.hpp"
#include "ublox_i2c_core/gnss_gate.hpp"
#include <unistd.h>
#include <vector>
#include <deque>
#include <string>
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

        // Multipath mitigation: nav_filter.* / diag.* / gate.* (config/multipath.yaml).
        declare_multipath_params();

        send_ubx_set_uart2_baud(460800);
        send_ubx_cfg_rate(40, 5);
        send_ubx_enable_uart2_ubx_out();
        send_ubx_enable_uart2_rtcm3_out();
        send_ubx_try_enable_4072_0_uart2();
        send_ubx_disable_rtcm3_input();
        send_ubx_nav_filter_config();
        if (use_nav_cov_) send_ubx_enable_nav_cov();
        if (nav_sat_enable_) send_ubx_enable_nav_sat(static_cast<uint8_t>(nav_sat_rate_));

        // Identify the receiver firmware in the log, then read back what the
        // multipath masks actually took (an ACK only means the frame was accepted).
        i2c_->write_ubx(ubx::build_mon_ver_poll());
        usleep(150000);
        poll_cfg_readback();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&BaseNode::loop, this));

        RCLCPP_INFO(get_logger(), "Base node ready. CFG-RATE meas=40ms nav=5. UART2 UBX+RTCM3 out enabled, RTCM3 input disabled. NAV-COV %s. Publishing /gps/fix + /gps/ubx_nav_pvt.",
            use_nav_cov_ ? "enabled" : "disabled");
        RCLCPP_INFO(get_logger(), "Output gate %s: hAcc<=%.1fm pDOP<=%.1f jump<=%.1fm/s.",
            gate_.params().enable ? "on" : "off", gate_.params().max_hacc_m,
            gate_.params().max_pdop, gate_.params().max_jump_speed_mps);
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
        pending_valset_.push_back("CFG-RATE-MEAS/NAV");
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
        pending_valset_.push_back("CFG-UART2-BAUDRATE");
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
        pending_valset_.push_back("CFG-UART2OUTPROT-UBX");
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
        pending_valset_.push_back("CFG-UART2 RTCM3X-out batch");
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
        pending_valset_.push_back("CFG-MSGOUT-RTCM_4072_0 probe");
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
        pending_valset_.push_back("CFG-*INPROT-RTCM3X disable");
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
        pending_valset_.push_back("CFG-MSGOUT-UBX_NAV_COV_I2C");
        usleep(150000);
    }

    // CFG-MSGOUT-UBX_NAV_SAT_I2C: UBX-NAV-SAT on I2C once every `rate` nav epochs.
    // Diagnostic only (diag.nav_sat_enable); ~392 bytes per message at 32 SVs on a
    // 100 kHz I2C bus shared with NAV-PVT, so keep it off unless diagnosing pDOP.
    void send_ubx_enable_nav_sat(uint8_t rate) {
        apply_cfg_key("CFG-MSGOUT-UBX_NAV_SAT_I2C", ubx::CFG_MSGOUT_UBX_NAV_SAT_I2C, rate);
    }

    // Multipath mitigation parameters (config/multipath.yaml).
    //   nav_filter.* -> UBX-CFG-NAVSPG masks applied to the receiver at startup.
    //   diag.*       -> optional UBX-NAV-SAT diagnostics.
    //   gate.*       -> output-side quality / jump gate (gnss::Gate).
    // Values are read once at startup; changing them needs a node restart, since
    // the receiver masks are only sent from the constructor.
    // Defaults here are the conservative built-ins; the shipped YAML overrides them.
    void declare_multipath_params() {
        nav_filter_enable_ = this->declare_parameter<bool>("nav_filter.enable", true);
        dyn_model_         = this->declare_parameter<int>("nav_filter.dyn_model", 3);
        min_elev_deg_      = this->declare_parameter<int>("nav_filter.min_elev_deg", 15);
        min_cno_dbhz_      = this->declare_parameter<int>("nav_filter.min_cno_dbhz", 15);
        outfil_pdop_       = this->declare_parameter<double>("nav_filter.outfil_pdop", 4.0);
        outfil_pacc_m_     = this->declare_parameter<int>("nav_filter.outfil_pacc_m", 30);

        nav_sat_enable_    = this->declare_parameter<bool>("diag.nav_sat_enable", false);
        nav_sat_rate_      = this->declare_parameter<int>("diag.nav_sat_rate", 5);

        // Gate thresholds. A threshold below the receiver's own baseline rejects every
        // epoch, so the gate fails open after max_reject_streak (see handle_pvt()).
        gnss::GateParams gp{};
        gp.enable              = this->declare_parameter<bool>("gate.enable", true);
        gp.require_fix_ok      = this->declare_parameter<bool>("gate.require_fix_ok", true);
        gp.max_hacc_m          = this->declare_parameter<double>("gate.max_hacc_m", 5.0);
        gp.max_pdop            = this->declare_parameter<double>("gate.max_pdop", 4.0);
        gp.max_jump_speed_mps  = this->declare_parameter<double>("gate.max_jump_speed_mps", 15.0);
        gp.jump_margin_sigma   = this->declare_parameter<double>("gate.jump_margin_sigma", 3.0);
        gp.jump_margin_m       = this->declare_parameter<double>("gate.jump_margin_m", 0.5);
        gp.jump_margin_max_m   = this->declare_parameter<double>("gate.jump_margin_max_m", 5.0);
        gp.max_reject_streak   = this->declare_parameter<int>("gate.max_reject_streak", 25);
        gp.float_cov_scale     = this->declare_parameter<double>("gate.float_cov_scale", 4.0);
        gp.spp_cov_scale       = this->declare_parameter<double>("gate.spp_cov_scale", 9.0);
        gp.fail_open_cov_scale = this->declare_parameter<double>("gate.fail_open_cov_scale", 25.0);
        gate_.set_params(gp);
    }

    // One key per UBX-CFG-VALSET (RAM layer), so a NAK on one key cannot take the
    // others down with it. The label is queued so handle_ack() can name the key
    // that each UBX-ACK-ACK / UBX-ACK-NAK belongs to. The receiver answers in
    // order, so a FIFO is enough. 150 ms between frames, as for every other VALSET
    // this node sends.
    void apply_cfg_key(const char *label, uint32_t key, uint64_t value) {
        i2c_->write_ubx(ubx::build_valset({{key, value}}, ubx::LAYER_RAM));
        pending_valset_.push_back(label);
        usleep(150000);
    }

    // UBX-CFG-NAVSPG multipath masks. These keep reflected signals out of the
    // navigation solution instead of filtering them after the fact:
    //   DYNMODEL       - platform model (3 = pedestrian; see multipath.yaml)
    //   INFIL_MINELEV  - elevation mask, deg (I1)
    //   INFIL_MINCNO   - C/N0 floor, dBHz (U1)
    //   OUTFIL_PDOP    - output pDOP limit, scaled x0.1 (U2)
    //   OUTFIL_PACC    - output position accuracy limit, m (U2)
    // Sent one key per CFG-VALSET; check the log for UBX-ACK-ACK / UBX-ACK-NAK.
    void send_ubx_nav_filter_config() {
        if (!nav_filter_enable_) {
            RCLCPP_WARN(get_logger(), "nav_filter.enable=false - receiver multipath masks NOT applied.");
            return;
        }

        // Platform model. Confirm via the CFG readback line, not the ACK.
        apply_cfg_key("CFG-NAVSPG-DYNMODEL", ubx::CFG_NAVSPG_DYNMODEL,
            static_cast<uint8_t>(dyn_model_));
        // Low-elevation satellites are the ones most often seen via reflections.
        apply_cfg_key("CFG-NAVSPG-INFIL_MINELEV", ubx::CFG_NAVSPG_INFIL_MINELEV,
            static_cast<uint8_t>(min_elev_deg_));
        // A reflected signal arrives attenuated: C/N0 floor is a direct discriminator.
        apply_cfg_key("CFG-NAVSPG-INFIL_MINCNO", ubx::CFG_NAVSPG_INFIL_MINCNO,
            static_cast<uint8_t>(min_cno_dbhz_));
        // Output filters: clear gnssFixOk on bad-geometry / low-accuracy epochs.
        apply_cfg_key("CFG-NAVSPG-OUTFIL_PDOP", ubx::CFG_NAVSPG_OUTFIL_PDOP,
            static_cast<uint16_t>(outfil_pdop_ * 10.0));
        // Position accuracy limit, metres.
        apply_cfg_key("CFG-NAVSPG-OUTFIL_PACC", ubx::CFG_NAVSPG_OUTFIL_PACC,
            static_cast<uint16_t>(outfil_pacc_m_));

        RCLCPP_INFO(get_logger(), "Nav filter sent: dynModel=%d minElev=%ddeg minCNO=%ddBHz outPDOP=%.1f outPACC=%dm. Watch for ACK/NAK.",
            dyn_model_, min_elev_deg_, min_cno_dbhz_, outfil_pdop_, outfil_pacc_m_);
    }

    // UBX-CFG-VALGET (0x06 0x8B): read back the multipath keys from the RAM layer,
    // one key per poll, so the log shows what the receiver actually holds. An ACK
    // only means the VALSET frame was accepted, not that the value took: a model the
    // receiver does not support is NAK'd and silently leaves the old value in place.
    void poll_cfg_readback() {
        static const uint32_t kKeys[] = {
            ubx::CFG_NAVSPG_DYNMODEL,
            ubx::CFG_NAVSPG_INFIL_MINELEV,
            ubx::CFG_NAVSPG_INFIL_MINCNO,
            ubx::CFG_NAVSPG_OUTFIL_PDOP,
            ubx::CFG_NAVSPG_OUTFIL_PACC,
        };
        // Responses arrive as UBX-CFG-VALGET (handled in handle_valget()) plus an
        // ACK/NAK each; queue the key name so the ACK can be attributed.
        for (uint32_t key : kKeys) {
            i2c_->write_ubx(ubx::build_valget({key}, ubx::VALGET_RAM, 0));
            pending_valget_.push_back(ubx::key_name(key));
            usleep(100000);
        }
    }

    // UBX-CFG-VALGET response: version(1) layer(1) position(2), then key/value pairs.
    // Each value's size comes from the key's size field; an unknown size ends the
    // walk, since the rest of the payload can no longer be framed.
    void handle_valget(size_t idx, uint16_t len) {
        if (len < 4) return;
        const size_t B = idx + 6;
        const uint8_t layer = buffer_[B + 1];

        size_t off = 4;
        while (off + 4 <= len) {
            uint32_t key = buffer_[B + off] | (buffer_[B + off + 1] << 8) |
                           (buffer_[B + off + 2] << 16) | (buffer_[B + off + 3] << 24);
            // Value follows the 4-byte key.
            size_t vsize = ubx::key_value_size(key);
            if (vsize == 0 || off + 4 + vsize > len) return;
            // Little-endian value of 1/2/4/8 bytes.
            uint64_t val = 0;
            for (size_t k = 0; k < vsize; k++)
                val |= static_cast<uint64_t>(buffer_[B + off + 4 + k]) << (8 * k);
            off += 4 + vsize;

            // Decode per key: E1 model, I1 deg, U1 dBHz, U2 x0.1, U2 m.
            const char *name  = ubx::key_name(key);
            const char *lname = ubx::layer_name(layer);
            if (key == ubx::CFG_NAVSPG_DYNMODEL) {
                RCLCPP_INFO(get_logger(), "CFG readback | %s [%s] = %u (%s)",
                    name, lname, static_cast<unsigned>(val), ubx::dyn_model_name(static_cast<uint8_t>(val)));
            } else if (key == ubx::CFG_NAVSPG_INFIL_MINELEV) {
                RCLCPP_INFO(get_logger(), "CFG readback | %s [%s] = %d deg",
                    name, lname, static_cast<int>(static_cast<int8_t>(val)));
            } else if (key == ubx::CFG_NAVSPG_INFIL_MINCNO) {
                RCLCPP_INFO(get_logger(), "CFG readback | %s [%s] = %u dBHz",
                    name, lname, static_cast<unsigned>(val));
            } else if (key == ubx::CFG_NAVSPG_OUTFIL_PDOP) {
                RCLCPP_INFO(get_logger(), "CFG readback | %s [%s] = %.1f",
                    name, lname, val / 10.0);
            } else if (key == ubx::CFG_NAVSPG_OUTFIL_PACC) {
                RCLCPP_INFO(get_logger(), "CFG readback | %s [%s] = %u m",
                    name, lname, static_cast<unsigned>(val));
            } else {
                RCLCPP_INFO(get_logger(), "CFG readback | key 0x%08X [%s] = %llu",
                    key, lname, static_cast<unsigned long long>(val));
            }
        }
    }

    // UBX-NAV-SAT (0x01 0x35): per-satellite usage. Diagnoses WHY pDOP is what it is:
    // which satellites were used in the solution and which carried corrections.
    // Payload: iTOW(4) version(1) numSvs(1) reserved(2), then 12 bytes per SV:
    // gnssId svId cno elev(I1) azim(I2) prRes(I2) flags(X4).
    void handle_nav_sat(size_t idx, uint16_t len) {
        if (len < 8) return;
        const size_t B = idx + 6;
        const uint8_t numSvs = buffer_[B + 1];
        if (8 + 12 * static_cast<size_t>(numSvs) > len) return;

        static const char *kGnss[8] = {"GPS", "SBAS", "GAL", "BDS", "IMES", "QZSS", "GLO", "NAVIC"};
        int used = 0, corrected = 0, below15 = 0;
        int used_per[8] = {0};
        int corr_per[8] = {0};
        int min_elev = 127, max_elev = -128;

        for (uint8_t i = 0; i < numSvs; i++) {
            const size_t S = B + 8 + 12 * static_cast<size_t>(i);
            uint8_t  gnssId = buffer_[S];
            int8_t   elev   = static_cast<int8_t>(buffer_[S + 3]);
            uint32_t flags  = buf_read<uint32_t>(buffer_, S + 8);
            bool sv_used = (flags & 0x08) != 0;
            // diffCorr, or any of sbas/rtcm/slas/spartn/pr/cr CorrUsed
            bool corr = (flags & 0x40) || (flags & 0x10000) || (flags & 0x20000) ||
                        (flags & 0x40000) || (flags & 0x80000) || (flags & 0x100000) ||
                        (flags & 0x200000);
            if (!sv_used) continue;

            used++;
            if (gnssId < 8) used_per[gnssId]++;
            if (elev < min_elev) min_elev = elev;
            if (elev > max_elev) max_elev = elev;
            if (elev < 15) below15++;
            if (corr) {
                corrected++;
                if (gnssId < 8) corr_per[gnssId]++;
            }
        }

        // e.g. "GPS:9/9 GAL:8/8 BDS:10/0"
        std::string per;
        for (int g = 0; g < 8; g++) {
            if (used_per[g] == 0) continue;
            if (!per.empty()) per += " ";
            per += std::string(kGnss[g]) + ":" + std::to_string(used_per[g]) + "/" + std::to_string(corr_per[g]);
        }


        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "NAV-SAT | tracked:%u used:%d corrected:%d | elev used %d..%d deg (below-15: %d) | per-GNSS used/corrected  %s",
            numSvs, used, corrected, min_elev, max_elev, below15, per.c_str());
    }

    // UBX-MON-VER (0x0A 0x04): swVersion[30] hwVersion[10] then extension[30] * N.
    void handle_mon_ver(size_t idx, uint16_t len) {
        const size_t B = idx + 6;
        if (len < 40) return;

        auto field = [&](size_t off, size_t maxlen) {
            std::string s;
            for (size_t k = 0; k < maxlen && buffer_[B + off + k] != 0; k++)
                s += static_cast<char>(buffer_[B + off + k]);
            return s;
        };

        RCLCPP_INFO(get_logger(), "MON-VER | sw=%s hw=%s", field(0, 30).c_str(), field(30, 10).c_str());
        for (size_t off = 40; off + 30 <= len; off += 30) {
            std::string ext = field(off, 30);
            if (!ext.empty())
                RCLCPP_INFO(get_logger(), "MON-VER | ext: %s", ext.c_str());
        }
    }

    // UBX-ACK-ACK (0x05 0x01) / UBX-ACK-NAK (0x05 0x00). The receiver answers
    // CFG-VALSET / CFG-VALGET in the order they were sent, so the oldest pending
    // label for that message id names the key being acknowledged.
    // Payload: clsID(1) msgID(1) of the acknowledged message. Only VALSET (0x8A)
    // and VALGET (0x8B) are tracked; this node sends no other CFG frames that
    // expect an ACK.
    // An ACK/NAK with no pending label (any other class/id, or an empty queue)
    // is still logged, as "(?)".
    void handle_ack(size_t idx, bool ack) {
        uint8_t cls = buffer_[idx + 6];
        uint8_t id  = buffer_[idx + 7];

        // Attribute the ACK/NAK to the oldest frame of that kind still waiting.
        std::string label = "?";
        if (cls == 0x06 && id == 0x8A && !pending_valset_.empty()) {
            label = pending_valset_.front();
            pending_valset_.pop_front();
        } else if (cls == 0x06 && id == 0x8B && !pending_valget_.empty()) {
            label = std::string("readback ") + pending_valget_.front();
            pending_valget_.pop_front();
        }

        // A NAK means the key was rejected and its mask is NOT in effect.
        if (ack)
            RCLCPP_INFO(get_logger(), "UBX-ACK-ACK  cls=0x%02X id=0x%02X (%s)", cls, id, label.c_str());
        else
            RCLCPP_ERROR(get_logger(), "UBX-ACK-NAK  cls=0x%02X id=0x%02X (%s) - key rejected, mask NOT applied", cls, id, label.c_str());
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
                        else if (cls == 0x05 && (id == 0x00 || id == 0x01)) handle_ack(i, id == 0x01);
                        else if (cls == 0x01 && id == 0x35 && nav_sat_enable_) handle_nav_sat(i, len);
                        else if (cls == 0x06 && id == 0x8B) handle_valget(i, len);
                        else if (cls == 0x0A && id == 0x04) handle_mon_ver(i, len);
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

    // UBX-NAV-PVT (0x01 0x07)
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
        double pdop  = pDOP * 0.01;

        // Output-side quality / jump gate. Drops fixes the receiver flagged as poor
        // and fixes implying impossible motion; accepted fixes get their covariance
        // scaled by solution type (gate.*_cov_scale).
        gnss::GateInput gin{};
        gin.lat_deg   = lat_raw * 1e-7;
        gin.lon_deg   = lon_raw * 1e-7;
        gin.t_sec     = this->now().seconds();
        gin.fix_ok    = gnssFixOk;
        gin.hacc_m    = hAcc_m;
        gin.pdop      = pdop;
        gin.carr_soln = carrSolnRaw;
        gin.num_sv    = fixType;
        const gnss::Verdict verdict = gate_.check(gin);
        const double cov_scale = gate_.cov_scale(carrSolnRaw);

        // NavSatFix
        auto fix_msg = sensor_msgs::msg::NavSatFix();
        fix_msg.header.stamp    = this->now();
        fix_msg.header.frame_id = "base_link";
        fix_msg.latitude   = lat_raw * 1e-7;
        fix_msg.longitude  = lon_raw * 1e-7;
        fix_msg.altitude   = hMSL / 1000.0;
        if (use_nav_cov_ && nav_cov_valid_ && nav_cov_itow_ == iTOW) {
            // Use the receiver's true ENU covariance from a same-epoch NAV-COV.
            for (size_t k = 0; k < 9; k++) fix_msg.position_covariance[k] = nav_cov_enu_[k] * cov_scale;
            fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
        } else {
            // Original isotropic hAcc/vAcc approximation.
            fix_msg.position_covariance[0] = hAcc_m * hAcc_m * cov_scale;
            fix_msg.position_covariance[4] = hAcc_m * hAcc_m * cov_scale;
            fix_msg.position_covariance[8] = vAcc_m * vAcc_m * cov_scale;
            fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        }
        if      (carrSolnRaw == 2) fix_msg.status.status = 2;
        else if (carrSolnRaw == 1) fix_msg.status.status = 1;
        else if (fixType     >= 3) fix_msg.status.status = 0;
        else                       fix_msg.status.status = -1;

        bool publish = gnss::accepted(verdict);
        if (!publish) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "BASE fix DROPPED (%s) | hAcc:%.2fm pDOP:%.2f sats:%u jump:%.2fm allowed-margin:%.2fm | dropped:%lu kept:%lu",
                gnss::verdict_str(verdict), hAcc_m, pdop, numSV, gate_.last_jump_m(),
                gate_.last_margin_m(), gate_.rejected_total(), gate_.accepted_total());
        } else if (verdict == gnss::Verdict::ReAnchor) {
            RCLCPP_WARN(get_logger(),
                "BASE gate RE-ANCHORED after %d consecutive jump rejections (last jump %.2fm) - position discontinuity is being passed through deliberately.",
                gate_.params().max_reject_streak, gate_.last_jump_m());
        } else if (verdict == gnss::Verdict::FailOpen) {
            // A hAcc/pDOP threshold that sits below this receiver's baseline would
            // reject every epoch forever. The gate fails open after a streak and the
            // fix is published with an inflated covariance - say so loudly, every
            // epoch, because the threshold needs retuning.
            // (Not throttled on purpose.)
            RCLCPP_ERROR(get_logger(),
                "BASE gate FAILING OPEN after %d consecutive threshold rejections | hAcc:%.2fm (limit %.2f) pDOP:%.2f (limit %.2f) | publishing with covariance x%.0f. A threshold is below this receiver's baseline - retune gate.max_hacc_m / gate.max_pdop.",
                gate_.params().max_reject_streak, hAcc_m, gate_.params().max_hacc_m,
                pdop, gate_.params().max_pdop, gate_.params().fail_open_cov_scale);
        }

        // A gated-out fix is never published; NAV-COV mode additionally suppresses
        // a bit-identical covariance.
        if (publish && use_nav_cov_) {
            // Duplicate-covariance guard: the ZED SDK rejects repeated covariance values
            // ("GNSS DATA COVARIANCE MUST VARY"), so skip publishing a bit-identical matrix.
            bool cov_identical = true;
            for (size_t k = 0; k < 9; k++) {
                if (fix_msg.position_covariance[k] != last_pub_cov_[k]) { cov_identical = false; break; }
            }
            if (cov_identical) {
                RCLCPP_DEBUG(get_logger(), "Skipping /gps/fix publish: covariance identical to previous epoch.");
                publish = false;  // skip publish
            } else {
                for (size_t k = 0; k < 9; k++) last_pub_cov_[k] = fix_msg.position_covariance[k];
            }
        }
        if (publish) fix_pub_->publish(fix_msg);

        // UBXNavPVT — exact field names from message definition
        // (published for every epoch, gated or not)
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
        // Append the gate verdict, e.g. " | gate: accept".
        size_t n = strlen(buf);
        snprintf(buf + n, sizeof(buf) - n, " | gate: %s", gnss::verdict_str(verdict));
        auto info_msg = std_msgs::msg::String();
        info_msg.data = buf;
        info_pub_->publish(info_msg);

        // Throttled console summary.
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "%s | Fix:%u CarrSoln:%u HeadVeh:%.2fdeg", buf, fixType, carrSolnRaw, headVeh * 1e-5);
    }

    std::vector<uint8_t> buffer_;
    std::unique_ptr<UbloxI2C> i2c_;

    bool use_nav_cov_ = false;  // parameter: gate all NAV-COV behavior (default = original)

    // Output-side quality / jump gate (gate.* parameters).
    gnss::Gate gate_{gnss::GateParams()};

    // nav_filter.* parameters (UBX-CFG-NAVSPG multipath masks).
    bool   nav_filter_enable_ = true;
    int    dyn_model_         = 3;     // 3 = pedestrian
    int    min_elev_deg_      = 15;
    int    min_cno_dbhz_      = 15;
    double outfil_pdop_       = 4.0;
    // diag.* parameters (UBX-NAV-SAT).
    bool   nav_sat_enable_    = false;
    int    nav_sat_rate_      = 5;
    int    outfil_pacc_m_     = 30;

    // Labels of sent-but-unacknowledged CFG-VALSET / CFG-VALGET frames, oldest first.
    std::deque<std::string> pending_valset_;
    std::deque<std::string> pending_valget_;

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
