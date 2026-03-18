#include "rclcpp/rclcpp.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "ublox_i2c_core/ublox_i2c_handler.hpp" 

class I2CManagerNode : public rclcpp::Node {
public:
    I2CManagerNode() : Node("i2c_bridge") {
        i2c_ = std::make_unique<UbloxI2C>(7, 0x42); 
        
        RCLCPP_INFO(this->get_logger(), "I2C Manager: Using Extreme Chunking for 100kHz Bus.");
        
        // Subscribe to the NTRIP client's RTCM topic
        sub_ = this->create_subscription<rtcm_msgs::msg::Message>(
            "/ntrip_client/rtcm", 10,
            std::bind(&I2CManagerNode::rtcm_callback, this, std::placeholders::_1));
    }

private:
    void rtcm_callback(const rtcm_msgs::msg::Message::SharedPtr msg) {
        // 1. Physically write the data to the I2C bus
        i2c_->write_rtcm(msg->message);
        
        // 2. Throttle the print statement to only output once every 10,000 ms (10 seconds)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
            "📡 NTRIP -> I2C: Actively injecting RTCM data (Last packet: %zu bytes)", msg->message.size());
    }

    std::unique_ptr<UbloxI2C> i2c_;
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2CManagerNode>());
    rclcpp::shutdown();
    return 0;
}