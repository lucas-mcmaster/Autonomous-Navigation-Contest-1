#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <random>

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
inline double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Compile-time constants
constexpr float FRONT_ANGLE = -M_PI / 2.0f;
constexpr float LEFT_ANGLE  =  0.0f;
constexpr float RIGHT_ANGLE = -M_PI;
constexpr float FOV_HALF_WIDTH = 20.0f * M_PI / 180.0f;


class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node"),gen_(std::random_device{}()), 
                                rotation_dist_(10, 30),//initializing random number gen functions
                                sign_change_(0, 1) // 0 means negative, 1 means positive
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

        //Initializing random number generator


        // Initialize general timer and movement variables
        start_time_ = this->now();
        angular_ = 0.0;
        linear_ = 0.0;
        pos_x_=0.0;
        pos_y_=0.0;
        yaw_= 0.0;
        start_yaw_ = 0.0;
        start_pos_x_ = 0.0;
        start_pos_y_ = 0.0;
        target_rotation_ = 0.0;
        target_move_ = 0.0;
        random_rotate_counter_=0; //counter to implement an occasional random walk
        turning_ = false;
        moving_ = false;

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
        // Set starting min dist to high value to be overwritten on first iteration
        min_dist_out = std::numeric_limits<float>::infinity();

        // Counter variables to calculate min and avg distances
        float sum = 0.0f;
        int count = 0;

        // Start and end indices for FOV
        int start = std::max<int>(0, center_idx - fov_half_beams);
        int end   = std::min<int>(
            scan->ranges.size() - 1,
            center_idx + fov_half_beams);

        // Loop through beam indices for FOV
        for (int i = start; i <= end; ++i)
        {
            // Get current beam distance
            float r = scan->ranges[i];

            if (std::isfinite(r))
            {
                // Compare to current min dist and use 0.15 threshold to avoid fixed structural obstacles on robot
                if (r < min_dist_out && r >= 0.15)
                {
                    // Overwrite min dist if the current beam has a smaller dist
                    min_dist_out = r;
                }

                // Increment counters
                sum += r;
                count++;
            }
        }

        // Calculate avg distance for FOV
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
        //Extracting pos
        pos_x_= odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;

        yaw_ = tf2::getYaw(odom->pose.pose.orientation);

        //RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));
    }


    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
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

    // Main control loop
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

        // EXPLORATION CODE BEGINS HERE:

        // Check for any pressed bumpers and set the state accordingly
        bool any_bumper_pressed=false;
        for (const auto& [key, val] : bumpers_) {
            if (val) {
                any_bumper_pressed = true;
                break;
            }
        }

        // Log position, orientation, and min laser distances
        RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg, Minimum laser distance in front, left, and right: (%.2f, %.2f, %.2f)", pos_x_, pos_y_, yaw_, rad2deg(yaw_), min_front_dist_, min_left_dist_, min_right_dist_);
        
        // Priority 1a: finish any in-progress backing-up from bumper hit
        if (moving_) {
            // Check how much distance robot has moved
            double distance_moved = std::sqrt(
                std::pow(pos_x_ - start_pos_x_, 2) +
                std::pow(pos_y_ - start_pos_y_, 2)
            );

            // Obtain target distance to move
            double target_distance = std::abs(target_move_);

            if (distance_moved < target_distance) {
                // Continue moving until robot hits target distance
                linear_ = (target_move_ > 0.0) ? 0.1 : -0.1; //CHANGED TO 0.1 BECAUSE IF YOU BUMPED YOU ARE CLOSE TO WALL
                angular_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "Moving: %.3f / %.3f m",
                            distance_moved,
                            target_distance);
            } else {
                // Reached target distance, start moving forward again
                RCLCPP_INFO(this->get_logger(), "Reached 0.15m backup, resuming forward movement");
                moving_ = false;
                //Code to turn around after hitting wall - copied from below
                // If avg distance to the right >= the left, turn 90deg to the right
                if (avg_right_dist_ >= avg_left_dist_) {
                    target_rotation_ = deg2rad(-90.0); // right turn
                } 
                
                // If avg distance to the left > the right, turn 90deg to the left
                else {
                    target_rotation_ = deg2rad(90.0); // left turn
                }

                // Start turning sequence
                turning_ = true;
                linear_ = 0.0;
                angular_ = (target_rotation_ > 0.0) ? 1.0 : -1.0;
            }
        }
        // Priority 1b: finish any in-progress 15 or 90 deg turn
        else if (turning_) {
            // Check how much the robot has rotated
            double angle_rotated = normalizeAngle(yaw_ - start_yaw_);

            // Compare amount rotated to target rotation, if less, keep rotating
            if (std::abs(angle_rotated) < std::abs(target_rotation_)) {
                linear_ = 0.0;
                angular_ = (target_rotation_ > 0.0) ? 1.0 : -1.0;
                RCLCPP_INFO(this->get_logger(), "Rotating: %.3f / %.3f degrees",
                            rad2deg(std::abs(angle_rotated)),
                            rad2deg(std::abs(target_rotation_)));
            } else {
                // Reached target rotation, start moving forward again
                turning_ = false;
                linear_ = 0.25;
                angular_ = 0.0;
            }
        }

        // Priority 2: if obstacle to the left or right, start a 15 deg turn away from obstacle
        else if (!any_bumper_pressed && (min_left_dist_ < 0.4 || min_right_dist_ < 0.4)) {
            // Capture starting yaw (heading)
            start_yaw_ = yaw_;

            // If obstacle to the left within 0.4m, turn right 15deg
            if (min_left_dist_ < 0.4) {
                target_rotation_ = deg2rad(-15.0); // right turn
            } 
            
            // If obstacle to the right within 0.4m, turn left 15deg
            else {
                target_rotation_ = deg2rad(15.0);  // left turn
            }

            // Start turning sequence
            turning_ = true;
            linear_ = 0.0;
            angular_ = (target_rotation_ > 0.0) ? 1.0 : -1.0;
        }

        // Priority 3: if obstacle in front, decide which way to go based on lidar data (skewed to right)
        else if (!any_bumper_pressed && min_front_dist_ < 0.5) {
            // Capture starting yaw (heading)
            start_yaw_ = yaw_;

            // If avg distance to the right >= the left, turn 90deg to the right
            if (avg_right_dist_ >= avg_left_dist_) {
                target_rotation_ = deg2rad(-90.0); // right turn
            } 
            
            // If avg distance to the left > the right, turn 90deg to the left
            else {
                target_rotation_ = deg2rad(90.0); // left turn
            }

            //Adding random walk feature every 5 rotations --- adds a random degree of rotation between 10 and 30 degrees to target_rotation 
            random_rotate_counter_=random_rotate_counter_+1;
            if (random_rotate_counter_==5){
                int random_change_=rotation_dist_(gen_);
                if (sign_change_(gen_)==0)
                {
                    random_change_=random_change_*(-1); //made negative randomly
                }
                target_rotation_=target_rotation_ + deg2rad(random_change_);
                RCLCPP_INFO(this->get_logger(), "Random Rotation!");
                random_rotate_counter_=1;
            }
            // Start turning sequence
            turning_ = true;
            linear_ = 0.0;
            angular_ = (target_rotation_ > 0.0) ? 1.0 : -1.0;
        }

        // Priority 4: move forward if clear
        else if (!any_bumper_pressed && min_front_dist_ >= 0.5) {
            angular_ = 0.0;
            linear_ = 0.25;
        }

        // Priority 5: if bumper hit, back up 0.15 m
        else if (any_bumper_pressed) {
            // Record starting position
            start_pos_x_ = pos_x_;
            start_pos_y_ = pos_y_;

            // Set target moving distance
            target_move_ = -0.15;

            // Start backwards movement
            moving_ = true;
            turning_ = false;
            linear_ = (target_move_ > 0.0) ? 0.1 : -0.1; //Changed from 0.25 to 0.1 as if you hit bumper you must be close to wall
            angular_ = 0.0;
        }

        // Fallback for errors: stop moving robot
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
    double start_yaw_;
    double start_pos_x_;
    double start_pos_y_;
    double target_rotation_;
    double target_move_;
    bool turning_;
    bool moving_;
    int random_rotate_counter_;
    std::mt19937 gen_;
    std::uniform_int_distribution<int> rotation_dist_; //These are random number generators to flip between negative and positive
    std::uniform_int_distribution<int> sign_change_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}