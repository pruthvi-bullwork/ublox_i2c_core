#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <string>

// ============================================================
// PRODUCTION CONFIGURATION — tune these to your robot
// ============================================================
static constexpr double PHYSICAL_BASELINE_M = 1.05;  // measured antenna separation (m)
static constexpr double BASELINE_TOLERANCE_M = 0.10; // allow ±10cm noise
static constexpr double ACC_THRESHOLD_M      = 0.05; // reject if hAcc > 5cm
static constexpr int    LATCH_COUNT_REQUIRED = 3;    // consecutive good frames before publishing
// ============================================================

class RoverNode : public rclcpp::Node {
public:
    RoverNode() : Node("rover_node") {
        imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("/imu_gps/data", 10);
        fix_pub_  = create_publisher<sensor_msgs::msg::NavSatFix>("/rover/fix", 10);
        pvt_pub_  = create_publisher<std_msgs::msg::String>("/rover/nav_pvt", 10);
        info_pub_ = create_publisher<std_msgs::msg::String>("/rover/info", 10);

        // Subscribe to base position
        base_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                if (msg->status.status == 2) {
                    base_lat_   = msg->latitude;
                    base_lon_   = msg->longitude;
                    base_ready_ = true;
                }
            });

        declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
        std::string port = get_parameter("serial_port").as_string();

        if (!init_serial(port, 115200)) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", port.c_str());
            return;
        }

        send_ubx_cfg(0x01, 0x07, 0x01); // NAV-PVT @ 1Hz on UART1

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RoverNode::loop, this));

        RCLCPP_INFO(get_logger(),
            "Rover node ready on %s | baseline=%.2fm tol=%.2fm acc<%.2fm latch=%d",
            port.c_str(),
            PHYSICAL_BASELINE_M, BASELINE_TOLERANCE_M,
            ACC_THRESHOLD_M, LATCH_COUNT_REQUIRED);
    }

    ~RoverNode() { if (serial_fd_ >= 0) close(serial_fd_); }

private:
    int serial_fd_ = -1;
    std::vector<uint8_t> buffer_;

    // Base position
    double base_lat_{0.0}, base_lon_{0.0};
    bool   base_ready_{false};

    // Safety gate state
    double last_good_heading_{0.0};
    bool   heading_locked_{false};
    int    latch_counter_{0};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr          imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr    fix_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pvt_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          info_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr base_sub_;
    rclcpp::TimerBase::SharedPtr                                  timer_;

    bool init_serial(const std::string& port, int baud) {
        (void)baud;
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;
        cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;   tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;  tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|IXOFF|IXANY);
        tty.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
        return tcsetattr(serial_fd_, TCSANOW, &tty) == 0;
    }

    void send_ubx_cfg(uint8_t cls, uint8_t id, uint8_t rate) {
        std::vector<uint8_t> msg = {
            0xB5, 0x62, 0x06, 0x01,
            0x08, 0x00,
            cls, id,
            0x00, rate, 0x00, rate, 0x00, 0x00
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
        return (cka == buffer_[start + len + 6] &&
                ckb == buffer_[start + len + 7]);
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
                        if (buffer_[i+2] == 0x01 && buffer_[i+3] == 0x07)
                            handle_pvt(i);
                        i += total; continue;
                    }
                } else break;
            }
            i++;
        }
        if (i > 0) buffer_.erase(buffer_.begin(), buffer_.begin() + i);
        if (buffer_.size() > 8192) buffer_.clear();
    }

    void publish_imu(double bearing_rad, float hAcc_m, double dist_m) {
        sensor_msgs::msg::Imu imu;
        imu.header.stamp    = now();
        imu.header.frame_id = "rover_link";
        imu.orientation.x = 0.0;
        imu.orientation.y = 0.0;
        imu.orientation.z = std::sin(bearing_rad / 2.0);
        imu.orientation.w = std::cos(bearing_rad / 2.0);
        double acc_rad = (dist_m > 0.1) ? (hAcc_m / dist_m) : 0.1;
        imu.orientation_covariance.fill(0.0);
        imu.orientation_covariance[8] = acc_rad * acc_rad;
        imu.angular_velocity_covariance[0]    = -1.0;
        imu.linear_acceleration_covariance[0] = -1.0;
        imu_pub_->publish(imu);
    }

    void publish_info(const char * msg) {
        std_msgs::msg::String m;
        m.data = msg;
        info_pub_->publish(m);
    }

    void handle_pvt(size_t idx) {
        // --- Extract all NAV-PVT fields ---
        uint32_t iTOW     = *(uint32_t*)&buffer_[idx+6+0];
        uint16_t year     = *(uint16_t*)&buffer_[idx+6+4];
        uint8_t  month    =  buffer_[idx+6+6];
        uint8_t  day      =  buffer_[idx+6+7];
        uint8_t  hour     =  buffer_[idx+6+8];
        uint8_t  min      =  buffer_[idx+6+9];
        uint8_t  sec      =  buffer_[idx+6+10];
        uint8_t  fixType  =  buffer_[idx+6+20];
        uint8_t  flags    =  buffer_[idx+6+21];
        uint8_t  numSV    =  buffer_[idx+6+23];
        int32_t  lon_raw  = *(int32_t* )&buffer_[idx+6+24];
        int32_t  lat_raw  = *(int32_t* )&buffer_[idx+6+28];
        int32_t  hMSL_raw = *(int32_t* )&buffer_[idx+6+36];
        uint32_t hAcc_raw = *(uint32_t*)&buffer_[idx+6+40];
        uint32_t vAcc_raw = *(uint32_t*)&buffer_[idx+6+44];
        int32_t  gSpeed   = *(int32_t* )&buffer_[idx+6+60];
        uint8_t  carrSoln = (flags >> 6) & 0x03;

        float  hAcc_m    = hAcc_raw / 1000.0f;
        float  vAcc_m    = vAcc_raw / 1000.0f;
        double rover_lat = lat_raw * 1e-7;
        double rover_lon = lon_raw * 1e-7;

        // --- Always publish NavSatFix and nav_pvt regardless of gates ---
        sensor_msgs::msg::NavSatFix fix;
        fix.header.stamp    = now();
        fix.header.frame_id = "rover_link";
        fix.latitude   = rover_lat;
        fix.longitude  = rover_lon;
        fix.altitude   = hMSL_raw / 1000.0;
        fix.position_covariance[0] = hAcc_m * hAcc_m;
        fix.position_covariance[4] = hAcc_m * hAcc_m;
        fix.position_covariance[8] = vAcc_m * vAcc_m;
        fix.position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        if      (carrSoln == 2) fix.status.status = 2;
        else if (carrSoln == 1) fix.status.status = 1;
        else if (fixType  >= 3) fix.status.status = 0;
        else                    fix.status.status = -1;
        fix_pub_->publish(fix);

        char pvt_buf[512];
        snprintf(pvt_buf, sizeof(pvt_buf),
            "{\"iTOW\":%u,\"fixType\":%u,\"carrSoln\":%u,"
            "\"numSV\":%u,\"lat\":%.7f,\"lon\":%.7f,"
            "\"hMSL\":%.3f,\"hAcc\":%.3f,\"vAcc\":%.3f,"
            "\"gSpeed\":%.3f,"
            "\"time\":\"%04u-%02u-%02uT%02u:%02u:%02u\"}",
            iTOW, fixType, carrSoln, numSV,
            rover_lat, rover_lon,
            hMSL_raw / 1000.0, hAcc_m, vAcc_m,
            gSpeed / 1000.0f,
            year, month, day, hour, min, sec);
        std_msgs::msg::String pm;
        pm.data = pvt_buf;
        pvt_pub_->publish(pm);

        // ============================================================
        // SAFETY GATE 1 — Accuracy gate: reject poor RTK
        // ============================================================
        if (carrSoln != 2 || hAcc_m >= ACC_THRESHOLD_M || !base_ready_) {
            latch_counter_ = 0;  // reset latch on any bad frame
            char buf[160];
            if (!base_ready_) {
                snprintf(buf, sizeof(buf),
                    "WAITING | No base fix yet");
            } else {
                snprintf(buf, sizeof(buf),
                    "REJECTED | carrSoln=%u hAcc=%.3fm sats=%u",
                    carrSoln, hAcc_m, numSV);
            }
            publish_info(buf);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", buf);
            return;
        }

        // --- Compute heading (passed accuracy gate) ---
        double lat1 = base_lat_ * M_PI / 180.0;
        double lon1 = base_lon_ * M_PI / 180.0;
        double lat2 = rover_lat * M_PI / 180.0;
        double lon2 = rover_lon * M_PI / 180.0;

        double dLon = lon2 - lon1;
        double y = std::sin(dLon) * std::cos(lat2);
        double x = std::cos(lat1) * std::sin(lat2) -
                   std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
        double bearing_rad = std::atan2(y, x);
        double heading_deg = std::fmod(
            (bearing_rad * 180.0 / M_PI) + 360.0, 360.0);

        double dist_m = std::sqrt(
            std::pow((lat2 - lat1) * 6371000.0, 2) +
            std::pow((lon2 - lon1) * 6371000.0 * std::cos(lat1), 2));

        // ============================================================
        // SAFETY GATE 2 — Physical baseline gate: reject geometry lies
        // ============================================================
        if (std::abs(dist_m - PHYSICAL_BASELINE_M) > BASELINE_TOLERANCE_M) {
            latch_counter_ = 0;  // reset latch on geometry mismatch
            char buf[160];
            snprintf(buf, sizeof(buf),
                "REJECTED | Baseline mismatch got=%.3fm expected=%.3fm±%.3fm",
                dist_m, PHYSICAL_BASELINE_M, BASELINE_TOLERANCE_M);
            publish_info(buf);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", buf);
            return;
        }

        // ============================================================
        // SAFETY GATE 3 — Latch filter: wait for N consecutive good frames
        // ============================================================
        latch_counter_++;
        if (latch_counter_ < LATCH_COUNT_REQUIRED) {
            char buf[160];
            snprintf(buf, sizeof(buf),
                "LATCHING | %d/%d good frames | heading=%.2f deg",
                latch_counter_, LATCH_COUNT_REQUIRED, heading_deg);
            publish_info(buf);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", buf);
            return;
        }

        // ============================================================
        // ALL GATES PASSED — publish valid heading
        // ============================================================
        last_good_heading_ = heading_deg;
        heading_locked_    = true;

        publish_imu(bearing_rad, hAcc_m, dist_m);

        char buf[160];
        snprintf(buf, sizeof(buf),
            "ROVER FIXED | Heading: %.2f deg | Dist: %.3fm | hAcc: %.3fm | sats: %u",
            heading_deg, dist_m, hAcc_m, numSV);
        publish_info(buf);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", buf);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverNode>());
    rclcpp::shutdown();
    return 0;
}