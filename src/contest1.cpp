#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// TURTLE BOT COMMANDS: GAZEBO: ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite

using namespace std::chrono_literals;

// Utility functions
inline double rad2deg(double rad) {return rad*180.0/M_PI;}
inline double deg2rad(double deg) {return deg * M_PI / 180.0; }

// Compile-time constants
constexpr float FRONT_ANGLE = -M_PI / 2.0f;
constexpr float LEFT_ANGLE  =  0.0f;
constexpr float RIGHT_ANGLE = -M_PI;
constexpr float FOV_HALF_WIDTH = deg2rad(40.0f);


class Contest1Node : public rclcpp::Node

public:
    Contest1Node()
        : Node("contest1_node")
    {
        // Initialize publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));

        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::odomCallback, this, std::placeholders::_1));

        // Timer for main control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize general timer and movement variables
        start_time_ = this->now();
        angular_ = 0.0;
        linear_ = 0.0;
        pos_x_=0.0;
        pos_y_=0.0;
        yaw_= 0.0;

        // Initialize LiDAR variables
        nLasers_ = 0;

        min_front_dist_ = std::numeric_limits<float>::infinity();
        min_left_dist_  = std::numeric_limits<float>::infinity();
        min_right_dist_ = std::numeric_limits<float>::infinity();

        avg_front_dist_ = std::numeric_limits<float>::infinity();
        avg_left_dist_  = std::numeric_limits<float>::infinity();
        avg_right_dist_ = std::numeric_limits<float>::infinity();
        
        // Log start message
        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");

        //Initialize bumper states
        bumpers_["bump_front_left"]=false;
        bumpers_["bump_front_center"]=false;
        bumpers_["bump_front_right"]=false;
        bumpers_["bump_left"]=false;
        bumpers_["bump_right"]=false;
    }


private:
    // Helper function to calculate minimum distance within a field-of-view of the LIDAR
    void computeFovStats(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        uint32_t center_idx,
        uint32_t fov_half_beams,
        float &min_dist_out,
        float &avg_dist_out)
    {
        min_dist_out = std::numeric_limits<float>::infinity();

        float sum = 0.0f;
        int count = 0;

        int start = std::max<int>(0, center_idx - fov_half_beams);
        int end   = std::min<int>(
            scan->ranges.size() - 1,
            center_idx + fov_half_beams);

        for (int i = start; i <= end; ++i)
        {
            float r = scan->ranges[i];

            if (std::isfinite(r))
            {
                if (r < min_dist_out)
                {
                    min_dist_out = r;
                }

                sum += r;
                count++;
            }
        }

        if (count > 0)
        {
            avg_dist_out = sum / count;
        }
        else
        {
            avg_dist_out = std::numeric_limits<float>::infinity();
        }
    }


    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Number of laser beams in LIDAR frame
        nLasers_ = (scan->angle_max - scan->angle_min)/scan->angle_increment;

        // Number of laser beams in each FOV (all FOVs equal in angular size)
        uint32_t fov_half_beams = FOV_HALF_WIDTH / scan->angle_increment;

        //RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset %d", nLasers_, desiredNLasers_);

        // Array indices for each FOV
        uint32_t front_idx = (FRONT_ANGLE - scan->angle_min) / scan->angle_increment;
        uint32_t left_idx  = (LEFT_ANGLE  - scan->angle_min) / scan->angle_increment;
        uint32_t right_idx = (RIGHT_ANGLE - scan->angle_min) / scan->angle_increment;

        // Calculate minimum and average distances for each LIDAR FOV (front, left, right)
        computeFovStats(scan, front_idx, fov_half_beams, min_front_dist_, avg_front_dist_);
        computeFovStats(scan, left_idx, fov_half_beams, min_left_dist_, avg_left_dist_);
        computeFovStats(scan, right_idx, fov_half_beams, min_right_dist_, avg_right_dist_);
    }


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // implement your code here
        //Extracting pos
        pos_x_= odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;

        yaw_ = tf2::getYaw(odom->pose.pose.orientation);

        //RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));
    }


    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        // implement your code here
        //reset all bumpers to released state

        for (auto& [key, val] : bumpers_) {
            val=false;
        }

        //update bumper states based on current detections
        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
                RCLCPP_INFO(this->get_logger(), "Bumper pressed %s", detection.header.frame_id.c_str());
            }
        }
    }

    void controlLoop()
    {
        // Calculate elapsed time
        auto current_time = this->now();
        double seconds_elapsed = (current_time - start_time_).seconds();

        // Check if 480 seconds (8 minutes) have elapsed
        if (seconds_elapsed >= 480.0) {
            RCLCPP_INFO(this->get_logger(), "Contest time completed (480 seconds). Stopping robot.");

            // Stop the robot
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.0;
            vel_pub_->publish(vel);

            // Shutdown the node
            rclcpp::shutdown();
            return;
        }

        // Implement your exploration code here

        bool any_bumper_pressed=false;
        for (const auto& [key, val] : bumpers_) {
            if (val) {
                any_bumper_pressed = true;
                break;
            }
        RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg, Minimum laser distance in front, left, and right: (%.2f, %.2f, %.2f)", pos_x_, pos_y_, yaw_, rad2deg(yaw_), min_front_dist_, min_left_dist_, min_right_dist_);

        /*
        }
        if (pos_x_ <= 0.5 && yaw_ < M_PI / 12 && !any_bumper_pressed && minLaserDist_>=0.7) {
            angular_= 0.0;
            linear_= 0.2;
        }
        else if (yaw_ < M_PI / 2 && pos_x_ > 0.5 && !any_bumper_pressed && minLaserDist_>=0.5){
            angular_= M_PI/6;
            linear_= 0.0;
        }
        else if (minLaserDist_ < 1.0 && !any_bumper_pressed)
        {
            linear_=0.1;
            if(yaw_ < 17/36*M_PI || pos_x_>0.6)
            {
                angular_= M_PI/12;
            }
            else if (yaw_ < 19/36*M_PI || pos_x_ <0.4)
            {
                angular_=-M_PI /12;
            }
            else{
                angular_=0;
            }
        }
        */

        else {
            angular_ = 0.0;
            linear_ = 0.0;
            rclcpp::shutdown();
            return;
        }
        
        // Set velocity command
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;

        // Publish velocity command
        vel_pub_->publish(vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    float angular_;
    float linear_;
    double pos_x_;
    double pos_y_;
    double yaw_;
    std::map<std::string, bool>bumpers_;
    uint32_t nLasers_;
    float min_front_dist_;
    float min_left_dist_;
    float min_right_dist_;
    float avg_front_dist_;
    float avg_left_dist_;
    float avg_right_dist_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}