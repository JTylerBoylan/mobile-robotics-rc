#include "src/CYdLidar.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#define LIDAR_SIZE_MISMATCH_THROTTLE_MS 100
#define LIDAR_ANGLE_OUT_OF_RANGE_THROTTLE_MS 50

using namespace rclcpp_lifecycle;
using namespace rclcpp_lifecycle::node_interfaces;

class YDLidarNode : public LifecycleNode {

public:

    YDLidarNode() : LifecycleNode("ydlidar_ros2_node") {}

    // Called once at the node creation
    LifecycleNodeInterface::CallbackReturn on_configure(const State &) {
        RCLCPP_INFO(get_logger(), "Configuring...");

        // Initialize parameters with default values
        this->declare_parameter("device_port", "/dev/ydlidar");
        this->declare_parameter("ignore_array", "");
        this->declare_parameter("baud_rate", 128000);
        this->declare_parameter("lidar_type", 1);
        this->declare_parameter("device_type", 0);
        this->declare_parameter("sample_rate", 5);
        this->declare_parameter("abnormal_check_count", 4);
        this->declare_parameter("intensity_bit", 8);
        this->declare_parameter("fixed_resolution", true);
        this->declare_parameter("reversion", true);
        this->declare_parameter("inverted", true);
        this->declare_parameter("auto_reconnect", true);
        this->declare_parameter("is_single_channel", false);
        this->declare_parameter("intensity", false);
        this->declare_parameter("support_motor_dtr", true);
        this->declare_parameter("angle_max", 180.0f);
        this->declare_parameter("angle_min", -180.0f);
        this->declare_parameter("range_max", 12.0f);
        this->declare_parameter("range_min", 0.1f);
        this->declare_parameter("frequency", 10.0f);
        this->declare_parameter("frame_id", "laser");
        this->declare_parameter("publish_rate", 20.0f);

        // Get parameters from parameter server
        this->get_parameter("device_port", device_port_);
        this->get_parameter("ignore_array", ignore_array_);
        this->get_parameter("baud_rate", baud_rate_);
        this->get_parameter("lidar_type", lidar_type_);
        this->get_parameter("device_type", device_type_);
        this->get_parameter("sample_rate", sample_rate_);
        this->get_parameter("abnormal_check_count", abnormal_check_count_);
        this->get_parameter("intensity_bit", intensity_bit_);
        this->get_parameter("fixed_resolution", fixed_resolution_);
        this->get_parameter("reversion", reversion_);
        this->get_parameter("inverted", inverted_);
        this->get_parameter("auto_reconnect", auto_reconnect_);
        this->get_parameter("is_single_channel", is_single_channel_);
        this->get_parameter("intensity", intensity_);
        this->get_parameter("support_motor_dtr", support_motor_dtr_);
        this->get_parameter("angle_max", angle_max_);
        this->get_parameter("angle_min", angle_min_);
        this->get_parameter("range_max", range_max_);
        this->get_parameter("range_min", range_min_);
        this->get_parameter("frequency", frequency_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("publish_rate", publish_rate_);

        // YDLidar SDK options
        laser_.setlidaropt(LidarPropSerialPort, device_port_.c_str(), device_port_.size());
        laser_.setlidaropt(LidarPropIgnoreArray, ignore_array_.c_str(), ignore_array_.size());
        laser_.setlidaropt(LidarPropSerialBaudrate, &baud_rate_, sizeof(int));
        laser_.setlidaropt(LidarPropLidarType, &lidar_type_, sizeof(int));
        laser_.setlidaropt(LidarPropDeviceType, &device_type_, sizeof(int));
        laser_.setlidaropt(LidarPropSampleRate, &sample_rate_, sizeof(int));
        laser_.setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check_count_, sizeof(int));
        laser_.setlidaropt(LidarPropIntenstiyBit, &intensity_bit_, sizeof(int));
        laser_.setlidaropt(LidarPropFixedResolution, &fixed_resolution_, sizeof(bool));
        laser_.setlidaropt(LidarPropReversion, &reversion_, sizeof(bool));
        laser_.setlidaropt(LidarPropInverted, &inverted_, sizeof(bool));
        laser_.setlidaropt(LidarPropAutoReconnect, &auto_reconnect_, sizeof(bool));
        laser_.setlidaropt(LidarPropSingleChannel, &is_single_channel_, sizeof(bool));
        laser_.setlidaropt(LidarPropIntenstiy, &intensity_, sizeof(bool));
        laser_.setlidaropt(LidarPropSupportMotorDtrCtrl, &support_motor_dtr_, sizeof(bool));
        laser_.setlidaropt(LidarPropMaxAngle, &angle_max_, sizeof(float));
        laser_.setlidaropt(LidarPropMinAngle, &angle_min_, sizeof(float));
        laser_.setlidaropt(LidarPropMaxRange, &range_max_, sizeof(float));
        laser_.setlidaropt(LidarPropMinRange, &range_min_, sizeof(float));
        laser_.setlidaropt(LidarPropScanFrequency, &frequency_, sizeof(float));

        // Initialize lidar
        if (!laser_.initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing YDLidar: %s", laser_.DescribeError());
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // Initialize publisher and timer
        this->laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
        this->publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&YDLidarNode::publishLidarData_, this));
        this->publish_timer_->cancel(); // don't start until activate

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Called once after the configure transition has been triggered
    LifecycleNodeInterface::CallbackReturn on_activate(const State &) {
        RCLCPP_INFO(get_logger(), "Activating...");
    
        // Turn on lidar
        if (!laser_.turnOn()) {
            RCLCPP_ERROR(this->get_logger(), "Error activating YDLidar: %s", laser_.DescribeError());
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // Start publishing
        this->publish_timer_->reset();

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Called once after the activate transition has been triggered
    LifecycleNodeInterface::CallbackReturn on_deactivate(const State &) {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        
        // Stop publishing
        this->publish_timer_->cancel();

        // Turn off lidar
        if (!laser_.turnOff()) {
            RCLCPP_ERROR(this->get_logger(), "Error deactivating YDLidar: %s", laser_.DescribeError());
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Called once after the deactivate transition has been triggered
    LifecycleNodeInterface::CallbackReturn on_cleanup(const State &) {
        RCLCPP_INFO(get_logger(), "Cleaning up...");

        // Disconnect lidar
        laser_.disconnecting();

        // Deconstruct ROS objects
        laser_publisher_ = nullptr;
        publish_timer_ = nullptr;

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Called once when the node is being shut down
    LifecycleNodeInterface::CallbackReturn on_shutdown(const State &) {
        RCLCPP_INFO(get_logger(), "Shutting down...");

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:

    CYdLidar laser_;

    // YDLidar params
    std::string device_port_;
    std::string ignore_array_;
    int baud_rate_;
    int lidar_type_;
    int device_type_;
    int sample_rate_;
    int abnormal_check_count_;
    int intensity_bit_;
    bool fixed_resolution_;
    bool reversion_;
    bool inverted_;
    bool auto_reconnect_;
    bool is_single_channel_;
    bool intensity_;
    bool support_motor_dtr_;
    float angle_max_;
    float angle_min_;
    float range_max_;
    float range_min_;
    float frequency_;

    // ROS params
    std::string frame_id_;
    float publish_rate_;

    // ROS publisher
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Send latest scan to ROS
    void publishLidarData_(){
        
        // Get latest scan
        LaserScan scan;
        if (!laser_.doProcessSimple(scan)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get scan.");
            return;
        }

        // Size check
        const float min_angle = scan.config.min_angle;
        const float max_angle = scan.config.max_angle;
        const float angle_increment = scan.config.angle_increment;
        const size_t scan_size = (max_angle - min_angle)/angle_increment + 1;
        if (scan_size != scan.points.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), LIDAR_SIZE_MISMATCH_THROTTLE_MS,
                "Lidar scan size mismatch. Check angle max, min, and increment options.");
        }

        // Convert scan to ROS msg
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp.sec = RCL_NS_TO_S(scan.stamp);
        scan_msg.header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg.header.stamp.sec);
        scan_msg.header.frame_id = frame_id_;
        scan_msg.angle_min = min_angle;
        scan_msg.angle_max = max_angle;
        scan_msg.angle_increment = angle_increment;
        scan_msg.scan_time = scan.config.scan_time;
        scan_msg.time_increment = scan.config.time_increment;
        scan_msg.range_min = scan.config.min_range;
        scan_msg.range_max = scan.config.max_range;

        scan_msg.ranges.resize(scan_size);
        scan_msg.intensities.resize(scan_size);
        for (auto iter = scan.points.cbegin(); iter != scan.points.cend(); ++iter) {
            const float angle = iter->angle;
            if (angle > max_angle || angle < min_angle) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), LIDAR_ANGLE_OUT_OF_RANGE_THROTTLE_MS,
                    "Lidar point out of range. Check angle max, min, and increment options.");
                continue;
            }
            const unsigned int idx = std::ceil((iter->angle - min_angle)/angle_increment);
            scan_msg.ranges[idx] = iter->range;
            scan_msg.intensities[idx] = iter->intensity;
        }

        // Publish to ROS
        laser_publisher_->publish(scan_msg);
    }

};

// ROS entry point
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YDLidarNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}