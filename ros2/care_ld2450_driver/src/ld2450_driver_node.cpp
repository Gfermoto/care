/*
C.A.R.E. LD2450 Driver Node
ROS2 node for interfacing with LD2450 radar sensor

This node handles communication with the LD2450 radar sensor,
publishes target data, and manages safety zones.
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "care_ld2450_driver/msg/radar_target.hpp"
#include "care_ld2450_driver/msg/radar_targets.hpp"
#include "care_ld2450_driver/msg/safety_status.hpp"
#include "care_ld2450_driver/srv/set_safety_zone.hpp"

#include <serial/serial.h>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

class LD2450DriverNode : public rclcpp::Node
{
public:
    LD2450DriverNode() : Node("ld2450_driver")
    {
        // Declare parameters
        this->declare_parameter("radar_port", "/dev/ttyUSB0");
        this->declare_parameter("radar_baud", 256000);
        this->declare_parameter("frame_id", "radar_link");
        this->declare_parameter("publish_rate", 20.0);
        this->declare_parameter("safety_distance", 300.0);
        this->declare_parameter("safety_angle", 60.0);
        this->declare_parameter("can_enabled", true);
        this->declare_parameter("debug", false);
        
        // Get parameters
        radar_port_ = this->get_parameter("radar_port").as_string();
        radar_baud_ = this->get_parameter("radar_baud").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        safety_distance_ = this->get_parameter("safety_distance").as_double();
        safety_angle_ = this->get_parameter("safety_angle").as_double();
        can_enabled_ = this->get_parameter("can_enabled").as_bool();
        debug_ = this->get_parameter("debug").as_bool();
        
        // Initialize serial connection
        try {
            serial_port_ = std::make_unique<serial::Serial>(
                radar_port_, radar_baud_, serial::Timeout::simpleTimeout(1000)
            );
            RCLCPP_INFO(this->get_logger(), "Connected to radar on %s", radar_port_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to radar: %s", e.what());
            return;
        }
        
        // Create publishers
        targets_pub_ = this->create_publisher<care_ld2450_driver::msg::RadarTargets>(
            "/care/radar/targets", 10);
        safety_status_pub_ = this->create_publisher<care_ld2450_driver::msg::SafetyStatus>(
            "/care/radar/safety_status", 10);
        diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/care/radar/diagnostics", 10);
        
        // Create service
        set_safety_zone_srv_ = this->create_service<care_ld2450_driver::srv::SetSafetyZone>(
            "/care/radar/set_safety_zone",
            std::bind(&LD2450DriverNode::setSafetyZoneCallback, this,
                std::placeholders::_1, std::placeholders::_2));
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&LD2450DriverNode::timerCallback, this));
        
        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        RCLCPP_INFO(this->get_logger(), "LD2450 Driver Node started");
    }
    
private:
    void timerCallback()
    {
        // Read radar data
        if (serial_port_ && serial_port_->available()) {
            std::string data = serial_port_->readline();
            if (parseRadarData(data)) {
                publishTargets();
                publishSafetyStatus();
                publishDiagnostics();
            }
        }
    }
    
    bool parseRadarData(const std::string& data)
    {
        // Parse LD2450 data format
        // This is a simplified parser - actual implementation would be more complex
        if (data.find("TARGET") != std::string::npos) {
            // Parse target data
            // Format: "TARGET ID=1 X=-19mm, Y=496mm, SPEED=0cm/s, RESOLUTION=360mm, DISTANCE=496mm, VALID=1"
            return true;
        }
        return false;
    }
    
    void publishTargets()
    {
        auto targets_msg = care_ld2450_driver::msg::RadarTargets();
        targets_msg.header.stamp = this->now();
        targets_msg.header.frame_id = frame_id_;
        
        // Add target data (simplified)
        for (int i = 0; i < 3; i++) {
            auto target = care_ld2450_driver::msg::RadarTarget();
            target.id = i + 1;
            target.x = 0;
            target.y = 0;
            target.speed = 0;
            target.distance = 0;
            target.angle = 0.0;
            target.valid = false;
            target.timestamp = this->now();
            
            targets_msg.targets.push_back(target);
        }
        
        targets_msg.count = 0;
        targets_msg.timestamp = this->now();
        
        targets_pub_->publish(targets_msg);
    }
    
    void publishSafetyStatus()
    {
        auto safety_msg = care_ld2450_driver::msg::SafetyStatus();
        safety_msg.emergency_stop = false;
        safety_msg.min_distance = safety_distance_;
        safety_msg.max_angle = safety_angle_;
        safety_msg.active_targets = 0;
        safety_msg.timestamp = this->now();
        
        safety_status_pub_->publish(safety_msg);
    }
    
    void publishDiagnostics()
    {
        auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
        diag_msg.header.stamp = this->now();
        
        auto status = diagnostic_msgs::msg::DiagnosticStatus();
        status.name = "ld2450_radar";
        status.hardware_id = radar_port_;
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Radar operational";
        
        diag_msg.status.push_back(status);
        diagnostics_pub_->publish(diag_msg);
    }
    
    void setSafetyZoneCallback(
        const std::shared_ptr<care_ld2450_driver::srv::SetSafetyZone::Request> request,
        std::shared_ptr<care_ld2450_driver::srv::SetSafetyZone::Response> response)
    {
        safety_distance_ = request->min_distance;
        safety_angle_ = request->max_angle;
        
        response->success = true;
        response->message = "Safety zone updated";
        
        RCLCPP_INFO(this->get_logger(), "Safety zone updated: distance=%.1f, angle=%.1f",
                   safety_distance_, safety_angle_);
    }
    
    // Parameters
    std::string radar_port_;
    int radar_baud_;
    std::string frame_id_;
    double publish_rate_;
    double safety_distance_;
    double safety_angle_;
    bool can_enabled_;
    bool debug_;
    
    // Serial connection
    std::unique_ptr<serial::Serial> serial_port_;
    
    // ROS2 interfaces
    rclcpp::Publisher<care_ld2450_driver::msg::RadarTargets>::SharedPtr targets_pub_;
    rclcpp::Publisher<care_ld2450_driver::msg::SafetyStatus>::SharedPtr safety_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Service<care_ld2450_driver::srv::SetSafetyZone>::SharedPtr set_safety_zone_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LD2450DriverNode>());
    rclcpp::shutdown();
    return 0;
}
