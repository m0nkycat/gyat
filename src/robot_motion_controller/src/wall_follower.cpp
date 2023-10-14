#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358

class WallFollower : public rclcpp::Node
{
public:
    WallFollower(): Node("wall_follower")
    {
        // TODO
        // 1. Declare all parameters
        this->declare_parameter<float>("following_distance", 0.7);
        this->declare_parameter<float>("following_angle", 0.0);
        this->declare_parameter<float>("angle_control_gain", 1.0);
        this->declare_parameter<float>("distance_control_gain", 0.5);
        
        // 2. Get all parameter values
        this->get_parameter("following_distance", following_distance_);
        this->get_parameter("following_angle", following_angle_);
        this->get_parameter("angle_control_gain", angle_control_gain_);
        this->get_parameter("distance_control_gain", distance_control_gain_);
        
        // 3. Print all parameter values
        RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
        RCLCPP_INFO(this->get_logger(), "following_angle: %.2f", following_angle_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain: %.2f", angle_control_gain_);
        RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", distance_control_gain_);
        
        // 4. Set the value of "following_angle_" after initialising all parameters
        following_angle_ = PI/2;  // Assuming wall to be followed is on the left

        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SystemDefaultsQoS());
        using namespace std::placeholders;
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollower::scan_callback, this, _1)
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    float following_distance_;
    float following_angle_;
    float angle_control_gain_;
    float distance_control_gain_;

    // TODO
    // Declare all private element variables to store parameters.
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // TODO
        // 1. Process the received scan_msg
        float min_value = std::numeric_limits<float>::infinity();
        int min_index = 0;
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            float distance = scan_msg->ranges[i];
            if (distance >= 0.2 && distance < min_value)  // ignoring any measurement less than 0.2 meter
            {
                min_value = distance;
                min_index = i;
            }
        }
        
        if (min_value == std::numeric_limits<float>::infinity())
        {
            RCLCPP_INFO(this->get_logger(), "No valid object detected");
            return;
        }

        float angle = scan_msg->angle_min + scan_msg->angle_increment * min_index;
        
        // 3. Write a Wall Follow Reactive Control
        geometry_msgs::msg::Twist cmd_vel_msg;
        if(min_value < 12)
        {
            if(min_value > (following_distance_ + 0.1))  // Assuming buffer zone to be 0.1
            {
                if(abs(angle) > PI/4.0)
                {
                    if(angle > PI/4.0)
                        cmd_vel_msg.angular.z = 1.0;
                    else
                        cmd_vel_msg.angular.z = -1.0;
                }
                else
                {
                    cmd_vel_msg.angular.z = 0;
                    cmd_vel_msg.linear.x = 0.4;  // Assuming forward_velocity to be 0.4
                }
            }
            else
            {
                cmd_vel_msg.angular.z = angle_control_gain_ * (angle - following_angle_);
                cmd_vel_msg.linear.x = 0.4 + distance_control_gain_ * (min_value - following_distance_);  // Assuming forward_velocity to be 0.4
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No Object is Detected");
            cmd_vel_msg.linear.x = 0.2;
        }
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}

