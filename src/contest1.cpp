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

//TURTLE BOT COMMANDS: GAZEBO: ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite

using namespace std::chrono_literals;

//Utility functions
inline double rad2deg(double rad) {return rad*180.0/M_PI;}
inline double deg2rad(double deg) {return deg * M_PI / 180.0; }

class Contest1Node : public rclcpp::Node
{
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

        // Initialize variables
        start_time_ = this->now();
        angular_ = 0.0;
        linear_ = 0.0;
        pos_x_=0.0;
        pos_y_=0.0;
        yaw_= 0.0;
        minLaserDist_=std::numeric_limits<float>::infinity();
        nLasers_=0;
        desiredNLasers_=0;
        desiredAngle_=45;
        minLaserDist_= std::numeric_limits<float>::infinity();
        groupedRange_ = {0,0,0,0};
        maxGroupedRange_= {0,0,0,0};
        maxAverageLaserDistance=0;
        directionSelection=0; //once the max average distance is found must calculate the angle
        laserIdxDirection=0;  //this gives the index then the direction must be calculated using this index

        

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
        //Initialize bumper states
        bumpers_["bump_front_left"]=false;
        bumpers_["bump_front_center"]=false;
        bumpers_["bump_front_right"]=false;
        bumpers_["bump_left"]=false;
        bumpers_["bump_right"]=false;
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // implement your code here
        nLasers_ = (scan->angle_max - scan->angle_min)/scan->angle_increment;
        laserRange_= scan->ranges;
        desiredNLasers_= deg2rad(desiredAngle_)/scan->angle_increment;
        //RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset %d", nLasers_, desiredNLasers_);
        float laser_offset = deg2rad(-180.0); //forcing front index to be the head (0 deg) angle of the robot
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;

        minLaserDist_= std::numeric_limits<float>::infinity();
        groupedRange_ = {0,0,0,0};
        maxGroupedRange_= {0,0,0,0};
        maxAverageLaserDistance=0;
        directionSelection=0;

        //Find maximum laser distance within +/- desiredAngle from front center

        if (deg2rad(desiredAngle_) < scan->angle_max && deg2rad(desiredAngle_)>scan->angle_min)
        {
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx < front_idx + desiredNLasers_; ++laser_idx) //Code to get min distance and average of groups of 4
            {
                minLaserDist_=std::min(minLaserDist_, laserRange_[laser_idx]);
            
            }
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx < front_idx + desiredNLasers_; laser_idx=laser_idx+4) //Code to get min distance and average of groups of 4
            {
                //CODE TO FIND MAX OPEN SPACE FOR ROBOT TO GO TOWARDS
                std::copy(laserRange_.begin() + laser_idx, laserRange_.begin() + laser_idx+3, groupedRange_.begin());
                float sum=0;
                float average=0;
                for(int i=0; i<4; i++) //to find average
                {
                 sum=sum+groupedRange_[i];
                }
                average=sum/4;
                if (average>maxAverageLaserDistance)
                {
                    maxAverageLaserDistance=average;
                    std::copy(groupedRange_.begin(), groupedRange_.begin()+3, maxGroupedRange_.begin()); //kinda pointless but kept it anyways
                    laserIdxDirection=laser_idx+1; // this is the index of the direction which the bot will go. +1 as needs to be in the middle.
                }
            
            }

            directionSelection=scan->angle_max-((laserIdxDirection+0.5)*scan->angle_increment);
        }
        else
        {
            for(uint32_t laser_idx=0; laser_idx < nLasers_; ++laser_idx)
            {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        }
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
        RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg, Minimum laser distance: %.2f", pos_x_, pos_y_, yaw_, rad2deg(yaw_), minLaserDist_);

        }
        if (linear_==0.0 && !any_bumper_pressed && yaw_!=directionSelection) { //startup or when stopped robot goes towards open space direction
            angular_=0.5;
        }
        else if (linear_==0.0 && !any_bumper_pressed && minLaserDist_>=0.3){ //once direction is correct move forward
            angular_=0.0;
            linear_=0.1;
        }
        else if (linear_>0.0 && !any_bumper_pressed && minLaserDist_>=0.3)
        {
            linear_=0.1;
            angular_=0.0;
        }
        else if (linear_>0.0 && !any_bumper_pressed && minLaserDist_<0.3)
        {
            linear_=0.0;
            angular_=0.0;
        }
        else {
            angular_=0.0;
            linear_=0.0;
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
    float minLaserDist_;
    uint32_t nLasers_;
    uint32_t laserIdxDirection;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;
    std::vector<float> groupedRange_; //groups of laser distances in groups of 4 to take average distance
    std::vector<float> maxGroupedRange_; // largest group of 4
    float averageLaserDistance;
    float maxAverageLaserDistance;
    float directionSelection; //The direction with the most open space for the robot to go
    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
