#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class NtripNode : public rclcpp::Node {
public:
    NtripNode() : Node("ntrip_node") {
        // NOTE: Capital 'I' in UInt8MultiArray
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/ntrip_client/rtcm", 10);
        RCLCPP_INFO(this->get_logger(), "NTRIP Node Started");
    }
private:
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NtripNode>());
    rclcpp::shutdown();
    return 0;
}