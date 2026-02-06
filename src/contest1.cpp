#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//Utility functions for conversions 
inline double rad2deg(double rad){
    return rad *180.0/M_PI;
}
inline double deg2rad(double deg){
    return deg* M_PI/180.0;
}
//Func to convert range array to [far,close....etc]
//Modifies the array in place (void return)
//edge case: invalid ranges 
void classifyRanges (const std::vector<float>& input_ranges, std::vector<float>& output_flags, int size, float threshold){
    for(int i=0; i<size; i++){
       const float r = input_ranges[i];
       if(!std::isfinite(r)){
        output_flags[i] = 1.00; //treat inf as open space
       }
       else if(r > threshold){
        output_flags[i] = 1.00; //far
       }
       else{
        output_flags[i] = 0.00; //close (includes NaN)
       }
}
}

//function to return the midpoint index of the longest sequence of far ranges
//edge cases to address: multiple long sequences of the same length
int midSequenceFar (const std::vector<float>& sequence, int size){
    
    int current = 0;
    int longest = 0;
    int last_index = 0;

    //loop to find the longest sequence of far ranges (range = 1.0)
    for(int i=0; i<size; i++){
        if(sequence[i] > 0.5 ){
            current ++;
            if(current > longest){
                last_index = i;
                longest = current;
            }
            else{
                continue;
            }
        }
        else{
            current = 0;
        }
    }
    //return -1 incase no sequence of far ranges found, otherwise return left mid point
    if(!longest){
        return -1;
    }
    else{
        int mid_index = last_index - longest/2;
        return mid_index;
    }
}
//return the center of the narrowest space of far range 
//Need to check that it is wide enough for the body of the turtlebot
//Turtlebot Body is 0.4 m wide 
int midSequenceClose (const std::vector<float>& sequence, int size, const float threshold, const float angle_increment){
    //minimum theta value for arc length > bot width 

    float angle_range = 0.5/threshold;
    int index_width = angle_range / angle_increment; 
    int current = 0;
    int shortest = 1000;
    int last_index = 0;

    //loop to find the shortest sequence of far ranges (range = 1.0)
    for(int i=0; i<size; i++){
        if(sequence[i] > 0.5 ){
            current ++;
            if(current < shortest && current > index_width){
                last_index = i;
                shortest = current;
            }
            else{
                continue;
            }
        }
        else{
            current = 0;
        }
    }
    //return -1 incase no sequence of far ranges found, otherwise return left mid point
    if(!shortest){
        return -1;
    }
    else{
        int mid_index = last_index - shortest/1.5;
        return mid_index;
    }
}

//function to return the maximum range found from laser scan 
float maxRange(const std::vector<float>& ranges) {
  float max_r = 0.0f;
  for (float r : ranges) {
    if (std::isfinite(r)) {
      max_r = std::max(max_r, r);
    }
  }
  return max_r;
}

float minRangeFromIndex(const std::vector<float>& ranges, const int mid_index) {
  float min = 12.0 ; 
  for (int i = mid_index - 5; i < mid_index + 5; i++) {
    if (ranges[i] < min) {
      min = ranges[i];
    }
  }
  return min;
}



//Given an index from laser scan, returns yaw with repsect to front of robot
float LasertoFrontTF(const int mid_index, const float increment){
    float angle = rad2deg(increment*mid_index);
    if (angle <= 90){
        angle -= 90; 
    }
    else if (angle <= 270){
        angle -= 90;
    }
    else {
        angle -= 450;
    }

    return deg2rad(angle);
     
}
//Given an angle from [0,2pi], normalize to [-pi,pi]
//Since odom angle measurement is [-pi,pi]
float NormAngle(float angle){
    if(angle > M_PI){
        angle -= 2.0*M_PI;
    }
    else if(angle < -M_PI){
        angle += 2.0*M_PI;
    }
    return angle;
}


using namespace std::chrono_literals;

//function 
class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")
    {
        // Initialize publisher for velocity commands
        //Creates a publisher object, "cmd_vel" topic, message type TwistStamped
        //msg to send linear and angular velocities 
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        
        //creates a subscriber object to scan topic, message type is Laser scan 
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));
        
        //creates a subscriber to hazard_detection topic, things like bumps, stall and so on 
        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));
        
        //subscribes to odom topic, receives odometry messages 
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
        
        //initialize odom 
        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;

        //Initialize bumper states 
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;

        //initialize laser variable 
        minLaserDist_ = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 5;
        bestAngle_= 0.0;
        bestClearance_ = 0.0;
        haveScan_ = false;

        // movement state initialization 
        start_yaw_ = 0.0;
        start_pos_x_ = 0.0;
        start_pos_y_ = 0.0;
        target_rotation_ = 0.0;
        target_move_ = 0.0;
        turning_ = false;
        moving_ = false;
        backing_ = false;
        
        //initialize Sequence
        far_sequence_ = false;
        midpoint_ = -1;

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {

        //determine number of lasers
        nLasers_= (scan->angle_max - scan->angle_min)/scan->angle_increment;
        

        laserRange_ = scan->ranges;
        //Threshold based on maximum laser distance
        float threshold = 0.7*maxRange(laserRange_); //Play around
             
        closeFar_.resize(laserRange_.size());

        classifyRanges(laserRange_, closeFar_, laserRange_.size(), threshold);
        
        if (!far_sequence_){
            midpoint_ = midSequenceFar(closeFar_, laserRange_.size());
            far_sequence_ = true;
            RCLCPP_INFO(this->get_logger(), "Choosing most wide open space");
        }
        else{
            midpoint_ = midSequenceClose(closeFar_, laserRange_.size(), threshold, scan->angle_increment);
            far_sequence_ = false;
            RCLCPP_INFO(this->get_logger(), "Choosing most narrow open space");

        }

        if (midpoint_ == -1) {
            //RCLCPP_INFO(this->get_logger(), "No open space found");
            //implement failsafe code 
        }   
        else{
            bestAngle_ = LasertoFrontTF(midpoint_, scan->angle_increment);
           //bestClearance_ = (std::isinf(laserRange_[midpoint_]))? 12.0: laserRange_[midpoint_];
            bestClearance_ = (minRangeFromIndex(laserRange_, midpoint_));
            haveScan_ = true;
            RCLCPP_INFO(this->get_logger(), "Angle of most open space: %f, Range of space: %f", rad2deg(bestAngle_), bestClearance_);
        }

    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // extract position from Odom message
        // x and y are the only valuable positons, rover does not move in the z direction

        // 
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;
        yaw_ = tf2::getYaw(odom->pose.pose.orientation); //in rad

        //RCLCPP_INFO(this->get_logger(), "Position: (%.2f,%.2f), Orientation %.2frad, %.2fdeg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));
    }

    /*Each time I get a hazard message, I look at all the hazards.
    If I see a left bumper hazard, I mark left bumper as pressed.
    If I see a right bumper hazard, I mark right bumper as pressed.*/
    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        for(auto& [key, val]: bumpers_){ //iterating through every key and val of bumper 
            val = false; //to reset all states back to false 
        }

        for(const auto& detection : hazard_vector->detections){ //detections contains detection objects,  

            if(detection.type == irobot_create_msgs::msg::HazardDetection::BUMP){
                bumpers_[detection.header.frame_id] = true;
                //general check for any bumper
                RCLCPP_INFO(this->get_logger(), "Bumper pressed: %s", detection.header.frame_id.c_str());
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

        //exploration + movement logic 

        // Check if any bumper pressed (reactive)
        bool any_bumper_pressed = false;
        for (const auto& [key, val] : bumpers_) {
            if (val) { any_bumper_pressed = true; break; }
        }

        // Priority 0: bumper preempts EVERYTHING: stop, cancel turn/move, back up 0.15m
        if (any_bumper_pressed && !backing_) {
            // immediate stop
            angular_ = 0.0;
            linear_ = 0.0;

            // cancel current actions
            turning_ = false;
            moving_ = false;

            // start backing up
            start_pos_x_ = pos_x_;
            start_pos_y_ = pos_y_;
            target_move_ = -0.15;
            backing_ = true;

            // command reverse
            linear_ = -0.1; //navigate at a speed of 0.1m/s when close to a wall
            angular_ = 0.0;
        }

        // If backing up is in progress, continue until 0.15m is reached
        // Need to implement a way to make sure backing does not cause a bump
        else if (backing_) {
            double distance_moved = std::sqrt(
                std::pow(pos_x_ - start_pos_x_, 2) +
                std::pow(pos_y_ - start_pos_y_, 2)
            );

            if (distance_moved < std::abs(target_move_)) {
                linear_ = -0.1;
                angular_ = 0.0;
            } else {
                // stop backing
                backing_ = false;
                linear_ = 0.0;
                angular_ = 0.0;
            }
        }

        // Finish turning in progress
        else if (turning_) {
            // how much rotated since start of turn
            double angle_rotated = NormAngle(yaw_ - start_yaw_);
            double tol = deg2rad(2.0); // small tolerance

            if (std::abs(angle_rotated) + tol < std::abs(target_rotation_)) {
                linear_ = 0.0;
                angular_ = (target_rotation_ > 0.0) ? 1.0 : -1.0;
            } else {
                // turn complete: start moving
                turning_ = false;

                start_pos_x_ = pos_x_;
                start_pos_y_ = pos_y_;

                // move 0.9 * bestClearance_ (captured at plan time)
                moving_ = true;

                linear_ = 0.25;
                angular_ = 0.0;
            }
        }

        // Finish moving in progress
        else if (moving_) {
            double distance_moved = std::sqrt(
                std::pow(pos_x_ - start_pos_x_, 2) +
                std::pow(pos_y_ - start_pos_y_, 2)
            );

            if (distance_moved < std::abs(target_move_)) {
                linear_ = 0.25;
                angular_ = 0.0;
            } else {
                // move complete
                moving_ = false;
                linear_ = 0.0;
                angular_ = 0.0;
            }
        }

        // plan a new “turn then move” segment
        else {
            if (!haveScan_) {
                // no scan yet -> stop
                linear_ = 0.0;
                angular_ = 0.0;

                //Implement failsafe code
            } else {
                // Plan the next segment:
                // 1) turn to bestAngle_
                // 2) move forward 0.9*bestClearance_

                start_yaw_ = yaw_;
                target_rotation_ = bestAngle_;

                // Capture distance NOW so it doesn't change mid-move
                target_move_ = 0.8 * bestClearance_; //Play Around

                // optional safety clamp (prevents absurd long moves)
                // if (target_move_ < 0.05) target_move_ = 0.05;
                // if (target_move_ > 2.0) target_move_ = 2.0;

                turning_ = true;
                moving_ = false;

                linear_ = 0.0;
                angular_ = (target_rotation_ > 0.0) ? 1.0 : -1.0;
            }
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
    //Initializing member variables to be accessible for both call backs and control loop
    float angular_;
    float linear_;
    //Odom
    double pos_x_;
    double pos_y_;
    double yaw_;
    //Hazard
    std::map<std::string, bool> bumpers_; //create a dictionary, key --> strings, values --> Boolean
    //Laser
    float minLaserDist_;
    int32_t nLasers_; //how many lasers there are
    int32_t desiredNLasers_; //desired index or specific laser
    int32_t desiredAngle_; //desired Yaw angle
    std::vector<float> laserRange_; 
    std::vector<float> closeFar_;
    float bestAngle_;
    float bestClearance_;
    bool haveScan_;

    // movement state 
    double start_yaw_;
    double start_pos_x_;
    double start_pos_y_;
    double target_rotation_;
    double target_move_;
    bool turning_;
    bool moving_;
    bool backing_;
    
    //Robot dimensions
    float bot_width_;
    
    //Sequence State 
    bool far_sequence_;
    int midpoint_;
    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}