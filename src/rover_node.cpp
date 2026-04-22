#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp"
#include "std_msgs/msg/string.hpp" 
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>

class RoverNode : public rclcpp::Node {
public:
    RoverNode() : Node("rover_node") {
        fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/rover/fix", 10);
        relposned_pub_ = create_publisher<ublox_ubx_msgs::msg::UBXNavRelPosNED>("/rover/ubx_nav_rel_pos_ned", 10);
        pvt_pub_ = create_publisher<std_msgs::msg::String>("/rover/nav_pvt", 10); 

        declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
        std::string port = get_parameter("serial_port").as_string();

        if (!init_serial(port, 115200)) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", port.c_str());
            return;
        }

        send_ubx_cfg_rate(500);         // Lock to 2Hz
        send_ubx_cfg(0x01, 0x07, 0x01); // Enable NAV-PVT
        send_ubx_cfg(0x01, 0x3C, 0x01); // Enable Hardware NAV-RELPOSNED
        
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoverNode::loop, this));

        RCLCPP_INFO(get_logger(), "Rover node ready. Hardware RELPOSNED Active. Dropping invalid headings.");
    }

    ~RoverNode() { if (serial_fd_ >= 0) close(serial_fd_); }

private:
    int serial_fd_ = -1;
    std::vector<uint8_t> buffer_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr relposned_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pvt_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;

    bool init_serial(const std::string& port, int baud) {
        (void)baud;
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;
        cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD | CS8);
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|IXOFF|IXANY);
        tty.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
        return tcsetattr(serial_fd_, TCSANOW, &tty) == 0;
    }

    void send_ubx_cfg_rate(uint16_t rate_ms) {
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00};
        msg.push_back(rate_ms & 0xFF); msg.push_back((rate_ms >> 8) & 0xFF);
        msg.push_back(0x01); msg.push_back(0x00); msg.push_back(0x01); msg.push_back(0x00); 
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        write(serial_fd_, msg.data(), msg.size());
        usleep(150000); 
    }

    void send_ubx_cfg(uint8_t cls, uint8_t id, uint8_t rate) {
        std::vector<uint8_t> msg = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, cls, id, 0x00, rate, 0x00, rate, 0x00, 0x00};
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
                uint16_t len = buffer_[i+4] | (buffer_[i+5] << 8);
                if (len > 1024) { i++; continue; }
                size_t total = 8 + len;
                if (i + total <= buffer_.size()) {
                    if (verify_ubx_checksum(i, len)) {
                        if (buffer_[i+2] == 0x01 && buffer_[i+3] == 0x07) handle_pvt(i);
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

    // Parses the true Hardware Vector payload
    void handle_relposned(size_t idx) {
        int32_t heading_raw = *(int32_t*)&buffer_[idx+6+24];
        uint32_t flags = *(uint32_t*)&buffer_[idx+6+60];

        ublox_ubx_msgs::msg::UBXNavRelPosNED msg;
        msg.header.stamp = now();
        msg.header.frame_id = "rover";
        
        msg.rel_pos_heading = heading_raw;
        
        // Extract boolean flags straight from the u-blox chip's bitmask
        msg.gnss_fix_ok = (flags & 0x01) != 0;
        msg.diff_soln = (flags & 0x02) != 0;
        msg.rel_pos_valid = (flags & 0x04) != 0;
        msg.carr_soln.status = (flags >> 3) & 0x03;
        msg.is_moving = (flags & 0x20) != 0;
        msg.rel_pos_heading_valid = (flags & 0x100) != 0; 

        double heading_deg = heading_raw * 1e-5;

        // ONLY publish to the EKF if the hardware confirms the heading is actually valid!
        if (msg.rel_pos_heading_valid) {
            relposned_pub_->publish(msg);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                "HARDWARE HEADING | Yaw: %06.2f deg | Valid: %d | RTK: %d", 
                heading_deg, msg.rel_pos_heading_valid, msg.carr_soln.status);
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                "HARDWARE HEADING | DROPPED (Waiting for Base RTCM on UART2)");
        }
    }

    void handle_pvt(size_t idx) {
        uint8_t fixType = buffer_[idx+6+20];
        uint8_t flags = buffer_[idx+6+21];
        uint8_t carrSoln = (flags >> 6) & 0x03;
        int32_t lon_raw = *(int32_t*)&buffer_[idx+6+24];
        int32_t lat_raw = *(int32_t*)&buffer_[idx+6+28];
        int32_t hMSL_raw = *(int32_t*)&buffer_[idx+6+36];
        uint32_t hAcc_raw = *(uint32_t*)&buffer_[idx+6+40];
        uint32_t vAcc_raw = *(uint32_t*)&buffer_[idx+6+44];
        
        float hAcc_m = hAcc_raw / 1000.0f;
        float vAcc_m = vAcc_raw / 1000.0f;

        uint32_t iTOW = *(uint32_t*)&buffer_[idx + 6 + 0];
        uint16_t year = *(uint16_t*)&buffer_[idx + 6 + 4];
        uint8_t month = buffer_[idx + 6 + 6];
        uint8_t day   = buffer_[idx + 6 + 7];
        uint8_t hour  = buffer_[idx + 6 + 8];
        uint8_t min   = buffer_[idx + 6 + 9];
        uint8_t sec   = buffer_[idx + 6 + 10];
        uint8_t numSV = buffer_[idx + 6 + 23];
        int32_t gSpeed_raw = *(int32_t*)&buffer_[idx + 6 + 60];

        char pvt_buf[512];
        snprintf(pvt_buf, sizeof(pvt_buf),
            "{\"iTOW\":%u,\"fixType\":%u,\"carrSoln\":%u,\"numSV\":%u,"
            "\"lat\":%.7f,\"lon\":%.7f,\"hMSL\":%.3f,\"hAcc\":%.3f,\"vAcc\":%.3f,"
            "\"gSpeed\":%.3f,\"time\":\"%04u-%02u-%02uT%02u:%02u:%02u\"}",
            iTOW, fixType, carrSoln, numSV, lat_raw * 1e-7, lon_raw * 1e-7,
            hMSL_raw / 1000.0, hAcc_m, vAcc_m, gSpeed_raw / 1000.0f,
            year, month, day, hour, min, sec);
        
        auto pvt_msg = std_msgs::msg::String();
        pvt_msg.data = pvt_buf;
        pvt_pub_->publish(pvt_msg);

        double rover_lat = lat_raw * 1e-7;
        double rover_lon = lon_raw * 1e-7;

        sensor_msgs::msg::NavSatFix fix;
        fix.header.stamp = now();
        fix.header.frame_id = "rover_link";
        fix.latitude = rover_lat;
        fix.longitude = rover_lon;
        fix.altitude = hMSL_raw / 1000.0;

        fix.position_covariance[0] = hAcc_m * hAcc_m;
        fix.position_covariance[4] = hAcc_m * hAcc_m;
        fix.position_covariance[8] = vAcc_m * vAcc_m; 
        fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        
        if (carrSoln == 2) fix.status.status = 2;
        else if (carrSoln == 1) fix.status.status = 1;
        else if (fixType >= 3) fix.status.status = 0;
        else fix.status.status = -1;
        fix_pub_->publish(fix);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverNode>());
    rclcpp::shutdown();
    return 0;
}
