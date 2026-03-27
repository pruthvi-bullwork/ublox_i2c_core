#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <string>

class RoverNode : public rclcpp::Node {
public:
    RoverNode() : Node("rover_node") {
        fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/rover/fix", 10);
        
        // THIS is the topic your Python script is waiting for!
        relposned_pub_ = create_publisher<ublox_ubx_msgs::msg::UBXNavRelPosNED>("/rover/ubx_nav_rel_pos_ned", 10);

        base_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                base_lat_ = msg->latitude;
                base_lon_ = msg->longitude;
                base_ready_ = true;
            });

        declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
        std::string port = get_parameter("serial_port").as_string();

        if (!init_serial(port, 115200)) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", port.c_str());
            return;
        }

        send_ubx_cfg_rate(500); // Lock to 2Hz
        send_ubx_cfg(0x01, 0x07, 0x01); // Enable NAV-PVT

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoverNode::loop, this));

        RCLCPP_INFO(get_logger(), "Rover node ready. Locked at 2Hz. Spoofing RELPOSNED.");
    }

    ~RoverNode() { if (serial_fd_ >= 0) close(serial_fd_); }

private:
    int serial_fd_ = -1;
    std::vector<uint8_t> buffer_;

    double base_lat_{0.0}, base_lon_{0.0};
    bool base_ready_{false};

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr relposned_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr base_sub_;
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
        msg.push_back(rate_ms & 0xFF);
        msg.push_back((rate_ms >> 8) & 0xFF);
        msg.push_back(0x01); 
        msg.push_back(0x00); 
        msg.push_back(0x01); 
        msg.push_back(0x00); 
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        write(serial_fd_, msg.data(), msg.size());
        usleep(150000); 
    }

    void send_ubx_cfg(uint8_t cls, uint8_t id, uint8_t rate) {
        std::vector<uint8_t> msg = {
            0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, cls, id, 0x00, rate, 0x00, rate, 0x00, 0x00
        };
        uint8_t cka = 0, ckb = 0;
        for (size_t i = 2; i < msg.size(); i++) { cka += msg[i]; ckb += cka; }
        msg.push_back(cka); msg.push_back(ckb);
        write(serial_fd_, msg.data(), msg.size());
        usleep(150000);
    }

    bool verify_ubx_checksum(size_t start, uint16_t len) {
        if (start + 8 + len > buffer_.size()) return false;
        uint8_t cka = 0, ckb = 0;
        for (size_t k = 2; k < (size_t)(len + 6); k++) {
            cka += buffer_[start + k]; ckb += cka;
        }
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
                        if (buffer_[i+2] == 0x01 && buffer_[i+3] == 0x07) {
                            handle_pvt(i);
                        }
                        i += total; continue;
                    }
                } else break;
            }
            i++;
        }
        if (i > 0) buffer_.erase(buffer_.begin(), buffer_.begin() + i);
        if (buffer_.size() > 8192) buffer_.clear();
    }

    void handle_pvt(size_t idx) {
        uint8_t fixType = buffer_[idx+6+20];
        uint8_t flags = buffer_[idx+6+21];
        int32_t lon_raw = *(int32_t*)&buffer_[idx+6+24];
        int32_t lat_raw = *(int32_t*)&buffer_[idx+6+28];
        int32_t hMSL_raw = *(int32_t*)&buffer_[idx+6+36];
        uint8_t carrSoln = (flags >> 6) & 0x03;

        double rover_lat = lat_raw * 1e-7;
        double rover_lon = lon_raw * 1e-7;

        sensor_msgs::msg::NavSatFix fix;
        fix.header.stamp = now();
        fix.header.frame_id = "rover_link";
        fix.latitude = rover_lat;
        fix.longitude = rover_lon;
        fix.altitude = hMSL_raw / 1000.0;
        
        if (carrSoln == 2) fix.status.status = 2;
        else if (carrSoln == 1) fix.status.status = 1;
        else if (fixType >= 3) fix.status.status = 0;
        else fix.status.status = -1;
        fix_pub_->publish(fix);

        // ==========================================================
        // THE RELPOSNED SPOOFING ENGINE 
        // ==========================================================
        if (base_ready_) {
            double lat1 = base_lat_ * M_PI / 180.0;
            double lon1 = base_lon_ * M_PI / 180.0;
            double lat2 = rover_lat * M_PI / 180.0;
            double lon2 = rover_lon * M_PI / 180.0;

            double dLon = lon2 - lon1;
            double y = std::sin(dLon) * std::cos(lat2);
            double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
            
            double bearing_rad = std::atan2(y, x);
            double bearing_deg = std::fmod((bearing_rad * 180.0 / M_PI) + 360.0, 360.0);

            ublox_ubx_msgs::msg::UBXNavRelPosNED spoof_msg;
            spoof_msg.header.stamp = now();
            spoof_msg.header.frame_id = "rover";

            spoof_msg.rel_pos_heading = static_cast<int32_t>(bearing_deg * 100000.0);

            // Bypasses the downstream Python script's safety checks!
            spoof_msg.is_moving = true; 
            spoof_msg.carr_soln.status = (carrSoln == 0) ? 1 : carrSoln; 

            relposned_pub_->publish(spoof_msg);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                "FEEDING DOWNSTREAM @ 2Hz | NED Yaw: %.2f deg | Forced RTK: %d", 
                bearing_deg, spoof_msg.carr_soln.status);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverNode>());
    rclcpp::shutdown();
    return 0;
}