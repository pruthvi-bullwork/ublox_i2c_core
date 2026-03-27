#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "ublox_i2c_core/ublox_i2c_handler.hpp"
#include <vector>

class BaseNode : public rclcpp::Node {
public:
    BaseNode() : Node("base_node") {
        fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        info_pub_ = this->create_publisher<std_msgs::msg::String>("/gps/info", 10);
        pvt_pub_ = this->create_publisher<std_msgs::msg::String>("/gps/nav_pvt", 10);
        i2c_ = std::make_unique<UbloxI2C>(7, 0x42);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&BaseNode::loop, this));
    }

private:
    bool verify_ubx_checksum(size_t start, uint16_t len) {
        if (start + len + 8 > buffer_.size()) return false;
        uint8_t cka = 0, ckb = 0;
        for (size_t k = 2; k < (size_t)(len + 6); k++) {
            cka += buffer_[start + k]; ckb += cka;
        }
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
                uint8_t cls = buffer_[i+2];
                uint8_t id = buffer_[i+3];
                uint16_t len = buffer_[i+4] | (buffer_[i+5] << 8);
                if (len > 1024) { i++; continue; }

                size_t total = 8 + len;

                if (i + total <= buffer_.size()) {
                    if (verify_ubx_checksum(i, len)) {
                        if (cls == 0x01 && id == 0x07) handle_pvt(i);
                        i += total;
                        continue;
                    }
                } else break;
            }
            else if (buffer_[i] == 0xD3 && i + 2 < buffer_.size()) {
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

    void handle_pvt(size_t index) {
        uint32_t iTOW = *(uint32_t*)&buffer_[index + 6 + 0];
        uint16_t year = *(uint16_t*)&buffer_[index + 6 + 4];
        uint8_t month = buffer_[index + 6 + 6];
        uint8_t day = buffer_[index + 6 + 7];
        uint8_t hour = buffer_[index + 6 + 8];
        uint8_t min = buffer_[index + 6 + 9];
        uint8_t sec = buffer_[index + 6 + 10];
        uint8_t fixType = buffer_[index + 6 + 20];
        uint8_t flags = buffer_[index + 6 + 21];
        uint8_t carrSoln = (flags >> 6) & 0x03;
        uint8_t numSV = buffer_[index + 6 + 23];
        int32_t lon_raw = *(int32_t*)&buffer_[index + 6 + 24];
        int32_t lat_raw = *(int32_t*)&buffer_[index + 6 + 28];
        int32_t hMSL_raw = *(int32_t*)&buffer_[index + 6 + 36];
        uint32_t hAcc_raw = *(uint32_t*)&buffer_[index + 6 + 40];
        uint32_t vAcc_raw = *(uint32_t*)&buffer_[index + 6 + 44];
        int32_t gSpeed_raw = *(int32_t*)&buffer_[index + 6 + 60];

        float hAcc_m = hAcc_raw / 1000.0f;
        float vAcc_m = vAcc_raw / 1000.0f;

        auto fix_msg = sensor_msgs::msg::NavSatFix();
        fix_msg.header.stamp = this->now();
        fix_msg.header.frame_id = "base_link";
        fix_msg.latitude = lat_raw * 1e-7;
        fix_msg.longitude = lon_raw * 1e-7;
        fix_msg.altitude = hMSL_raw / 1000.0;

        fix_msg.position_covariance[0] = hAcc_m * hAcc_m;
        fix_msg.position_covariance[4] = hAcc_m * hAcc_m;
        fix_msg.position_covariance[8] = vAcc_m * vAcc_m;
        fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        if (carrSoln == 2) fix_msg.status.status = 2;
        else if (carrSoln == 1) fix_msg.status.status = 1;
        else if (fixType >= 3) fix_msg.status.status = 0;
        else fix_msg.status.status = -1;
        fix_pub_->publish(fix_msg);

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

        auto info_msg = std_msgs::msg::String();
        char buf[128];
        if (carrSoln == 2) snprintf(buf, sizeof(buf), "BASE FIXED | Acc: %.4fm | Sats: %u", hAcc_m, numSV);
        else if (carrSoln == 1) snprintf(buf, sizeof(buf), "BASE FLOAT | Acc: %.4fm | Sats: %u", hAcc_m, numSV);
        else if (fixType >= 3) snprintf(buf, sizeof(buf), "BASE 3D FIX | Acc: %.4fm | Sats: %u", hAcc_m, numSV);
        else snprintf(buf, sizeof(buf), "BASE NO RTK | Acc: %.4fm | Sats: %u", hAcc_m, numSV);
        info_msg.data = buf;
        info_pub_->publish(info_msg);
    }

    std::vector<uint8_t> buffer_;
    std::unique_ptr<UbloxI2C> i2c_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pvt_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseNode>());
    rclcpp::shutdown();
    return 0;
}