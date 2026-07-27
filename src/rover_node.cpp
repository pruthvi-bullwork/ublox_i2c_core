#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pvt.hpp"
#include "std_msgs/msg/string.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <cstring>
#include <utility>

template<typename T>
static T buf_read(const std::vector<uint8_t>& buf, size_t offset) {
    T val; std::memcpy(&val, &buf[offset], sizeof(T)); return val;
}

class RoverNode : public rclcpp::Node {
public:
    RoverNode() : Node("rover_node") {
        fix_pub_       = create_publisher<sensor_msgs::msg::NavSatFix>("/rover/fix", 10);
        relposned_pub_ = create_publisher<ublox_ubx_msgs::msg::UBXNavRelPosNED>("/rover/ubx_nav_rel_pos_ned", 10);
        pvt_pub_       = create_publisher<ublox_ubx_msgs::msg::UBXNavPVT>("/rover/ubx_nav_pvt", 10);

        declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
        std::string port = get_parameter("serial_port").as_string();

        if (!init_serial(port, 115200)) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", port.c_str());
            return;
        }

        send_ubx_set_uart2_baud(460800);
        send_ubx_cfg_rate(40, 5);
        send_ubx_enable_uart1_messages();

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoverNode::loop, this));

        RCLCPP_INFO(get_logger(), "Rover node ready. CFG-RATE meas=40ms nav=5.");
    }

    ~RoverNode() { if (serial_fd_ >= 0) close(serial_fd_); }

private:
    int serial_fd_ = -1;
    std::vector<uint8_t> buffer_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr          fix_pub_;
    rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr relposned_pub_;
    rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr       pvt_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool init_serial(const std::string& port, int baud) {
        (void)baud;
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;
        cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
        tty.c_cflag |=  (CLOCAL | CREAD | CS8);
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|IXOFF|IXANY);
        tty.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
        return tcsetattr(serial_fd_, TCSANOW, &tty) == 0;
    }

    // UBX-CFG-VALSET (0x06 0x8A): explicitly set CFG-UART2-BAUDRATE (0x40530001, U4) to
    // match base_node's. This node talks to its receiver over UART1 (ttyTHS1), never UART2,
    // so changing it here is safe. A baud mismatch on the base-to-rover UART2 link would
    // explain why only short/simple RTCM messages ever survive on this rover while longer
    // MSM/4072.0 frames never do (short frames have much better odds of passing checksum by
    // chance under bit corruption from a baud mismatch).
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
        write(serial_fd_, msg.data(), msg.size());
        usleep(150000);
    }

    // UBX-CFG-VALSET (0x06 0x8A): set measurement + navigation rate via CFG-RATE-MEAS
    // (0x30210001, U2) and CFG-RATE-NAV (0x30210002, U2). Legacy UBX-CFG-RATE (0x06 0x08)
    // is confirmed UBX-ACK-NAK'd on this receiver (this X20P platform only implements
    // CFG-CFG/CFG-OTP/CFG-RST under class 0x06); keys verified against the ZED-X20P
    // Interface Description (UBXDOC-304424225-19888), section 6.9.17. Must match base_node's
    // rate exactly for moving-base processing to work.
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
        write(serial_fd_, msg.data(), msg.size());
        usleep(150000);
    }

    // UBX-CFG-VALSET (0x06 0x8A): enable NAV-PVT and NAV-RELPOSNED output on UART1 (the
    // port this node reads). Legacy UBX-CFG-MSG (0x06 0x01) is confirmed UBX-ACK-NAK'd on
    // this receiver, same platform limitation as CFG-RATE above.
    void send_ubx_enable_uart1_messages() {
        static const std::pair<uint32_t, uint8_t> kItems[] = {
            {0x20910007, 1},  // CFG-MSGOUT-UBX_NAV_PVT_UART1
            {0x2091008e, 1},  // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1
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
        write(serial_fd_, msg.data(), msg.size());
        usleep(150000);
    }

    bool verify_ubx_checksum(size_t start, uint16_t len) {
        if (start + 8 + len > buffer_.size()) return false;
        uint8_t cka = 0, ckb = 0;
        for (size_t k = 2; k < (size_t)(len + 6); k++) { cka += buffer_[start + k]; ckb += cka; }
        return (cka == buffer_[start + len + 6] && ckb == buffer_[start + len + 7]);
    }

    void loop() {
        if (serial_fd_ < 0) return;
        uint8_t read_buf[1024];
        int n = read(serial_fd_, read_buf, sizeof(read_buf));
        if (n > 0) buffer_.insert(buffer_.end(), read_buf, read_buf + n);

        size_t i = 0;
        while (i + 8 <= buffer_.size()) {
            if (buffer_[i] == 0xB5 && buffer_[i+1] == 0x62) {
                uint16_t len   = buffer_[i+4] | (buffer_[i+5] << 8);
                if (len > 1024) { i++; continue; }
                size_t   total = 8 + len;
                if (i + total <= buffer_.size()) {
                    if (verify_ubx_checksum(i, len)) {
                        if      (buffer_[i+2] == 0x01 && buffer_[i+3] == 0x07) handle_pvt(i);
                        else if (buffer_[i+2] == 0x01 && buffer_[i+3] == 0x3C) handle_relposned(i);
                        i += total; continue;
                    }
                } else break;
            }
            i++;
        }
        if (i > 0) buffer_.erase(buffer_.begin(), buffer_.begin() + i);
        if (buffer_.size() > 8192) buffer_.clear();
    }

    void handle_relposned(size_t idx) {
        const size_t P = idx + 6; // payload base

        // Full NAV-RELPOSNED v1 payload parsing
        uint8_t  version       = buffer_[P + 0];
        uint16_t refStationId  = buf_read<uint16_t>(buffer_, P + 2);
        uint32_t iTOW          = buf_read<uint32_t>(buffer_, P + 4);
        int32_t  relPosN       = buf_read<int32_t> (buffer_, P + 8);
        int32_t  relPosE       = buf_read<int32_t> (buffer_, P + 12);
        int32_t  relPosD       = buf_read<int32_t> (buffer_, P + 16);
        int32_t  relPosLength  = buf_read<int32_t> (buffer_, P + 20);
        int32_t  relPosHeading = buf_read<int32_t> (buffer_, P + 24);
        int8_t   relPosHPN     = (int8_t)buffer_[P + 32];
        int8_t   relPosHPE     = (int8_t)buffer_[P + 33];
        int8_t   relPosHPD     = (int8_t)buffer_[P + 34];
        int8_t   relPosHPLen   = (int8_t)buffer_[P + 35];
        uint32_t accN          = buf_read<uint32_t>(buffer_, P + 36);
        uint32_t accE          = buf_read<uint32_t>(buffer_, P + 40);
        uint32_t accD          = buf_read<uint32_t>(buffer_, P + 44);
        uint32_t accLength     = buf_read<uint32_t>(buffer_, P + 48);
        uint32_t accHeading    = buf_read<uint32_t>(buffer_, P + 52);
        uint32_t flags         = buf_read<uint32_t>(buffer_, P + 60);

        // flags breakdown
        bool    gnssFixOk          = (flags & 0x0001) != 0;
        bool    diffSoln           = (flags & 0x0002) != 0;
        bool    relPosValid        = (flags & 0x0004) != 0;
        uint8_t carrSolnRaw        = (flags >> 3) & 0x03;
        bool    isMoving           = (flags & 0x0020) != 0;
        bool    refPosMiss         = (flags & 0x0040) != 0;
        bool    refObsMiss         = (flags & 0x0080) != 0;
        bool    relPosHeadingValid = (flags & 0x0100) != 0;
        bool    relPosNormalized   = (flags & 0x0200) != 0;

        ublox_ubx_msgs::msg::UBXNavRelPosNED msg;
        msg.header.stamp    = now();
        msg.header.frame_id = "rover";

        msg.version            = version;
        msg.ref_station_id     = refStationId;
        msg.itow               = iTOW;
        msg.rel_pos_n          = relPosN;
        msg.rel_pos_e          = relPosE;
        msg.rel_pos_d          = relPosD;
        msg.rel_pos_length     = relPosLength;
        msg.rel_pos_heading    = relPosHeading;
        msg.rel_pos_hp_n       = relPosHPN;
        msg.rel_pos_hp_e       = relPosHPE;
        msg.rel_pos_hp_d       = relPosHPD;
        msg.rel_pos_hp_length  = relPosHPLen;
        msg.acc_n              = accN;
        msg.acc_e              = accE;
        msg.acc_d              = accD;
        msg.acc_length         = accLength;
        msg.acc_heading        = accHeading;
        msg.gnss_fix_ok        = gnssFixOk;
        msg.diff_soln          = diffSoln;
        msg.rel_pos_valid      = relPosValid;
        msg.carr_soln.status   = carrSolnRaw;
        msg.is_moving          = isMoving;
        msg.ref_pos_miss       = refPosMiss;
        msg.ref_obs_miss       = refObsMiss;
        msg.rel_pos_heading_valid = relPosHeadingValid;
        msg.rel_pos_normalized = relPosNormalized;

        double heading_deg = relPosHeading * 1e-5;
        double length_m    = (relPosLength + relPosHPLen * 0.01) * 0.01;
        double acc_len_m   = accLength * 0.0001;
        double acc_hdg_deg = accHeading * 1e-5;

        if (relPosHeadingValid) {
            relposned_pub_->publish(msg);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "HEADING | Yaw: %06.2f deg (acc: %.4f deg) | Length: %.3fm (acc: %.4fm) | RTK: %d",
                heading_deg, acc_hdg_deg, length_m, acc_len_m, carrSolnRaw);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "HARDWARE HEADING | DROPPED (Waiting for Base RTCM on UART2)");
        }
    }

    void handle_pvt(size_t idx) {
        const size_t B = idx + 6;

        // Raw byte parsing
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

        // flags byte breakdown
        bool     gnssFixOk  = (flags & 0x01) != 0;
        bool     diffSoln   = (flags & 0x02) != 0;
        uint8_t  psmState   = (flags >> 2) & 0x07;
        bool     headVehValid = (flags & 0x20) != 0;
        uint8_t  carrSolnRaw = (flags >> 6) & 0x03;

        // flags2 byte breakdown
        bool     confirmedAvail = (flags2 & 0x20) != 0;
        bool     confirmedDate  = (flags2 & 0x40) != 0;
        bool     confirmedTime  = (flags2 & 0x80) != 0;

        // valid byte breakdown
        bool     validDate     = (valid & 0x01) != 0;
        bool     validTime     = (valid & 0x02) != 0;
        bool     fullyResolved = (valid & 0x04) != 0;
        bool     validMag      = (valid & 0x08) != 0;

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
        sensor_msgs::msg::NavSatFix fix;
        fix.header.stamp    = now();
        fix.header.frame_id = "rover_link";
        fix.latitude   = lat_raw * 1e-7;
        fix.longitude  = lon_raw * 1e-7;
        fix.altitude   = hMSL / 1000.0;
        fix.position_covariance[0] = hAcc_m * hAcc_m;
        fix.position_covariance[4] = hAcc_m * hAcc_m;
        fix.position_covariance[8] = vAcc_m * vAcc_m;
        fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        if      (carrSolnRaw == 2) fix.status.status = 2;
        else if (carrSolnRaw == 1) fix.status.status = 1;
        else if (fixType     >= 3) fix.status.status = 0;
        else                       fix.status.status = -1;
        fix_pub_->publish(fix);

        // UBXNavPVT — mapped to exact field names from message definition
        auto pvt = ublox_ubx_msgs::msg::UBXNavPVT();
        pvt.header.stamp    = now();
        pvt.header.frame_id = "rover_link";

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

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "ROVER PVT | Fix:%u CarrSoln:%u Sats:%u Acc:%.3fm Speed:%.3fm/s HeadVeh:%.2fdeg",
            fixType, carrSolnRaw, numSV, hAcc_m, gSpeed / 1000.0f, headVeh * 1e-5);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverNode>());
    rclcpp::shutdown();
    return 0;
}
