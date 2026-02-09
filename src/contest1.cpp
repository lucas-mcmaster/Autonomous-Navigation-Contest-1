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

//Utility functions for conversions 
inline double rad2deg(double rad){
    return rad *180.0/M_PI;
}

inline double deg2rad(double deg){
    return deg* M_PI/180.0;
}

// Compile-time constants
constexpr float FRONT_ANGLE = -M_PI / 2.0f;
constexpr float LEFT_ANGLE  =  0.0f;
constexpr float RIGHT_ANGLE = -M_PI;
constexpr float FOV_HALF_WIDTH = 20.0f * M_PI / 180.0f;

//Func to convert range array to [far,close....etc]
//Modifies the array in place (void return)
//edge case: invalid ranges 
void classifyRanges (const std::vector<float>& input_ranges, std::vector<float>& output_flags, int size, float threshold){
    for(int i=0; i<size; i++){
       const float r = input_ranges[i];
       if(std::isnan(r)){
        output_flags[i] = 0.00; //treat NaN as close/unknown
       }
       else if(std::isinf(r)){
        output_flags[i] = 1.00; //treat inf as open space
       }
       else if(r > threshold){
        output_flags[i] = 1.00; //far
       }
       else{
        output_flags[i] = 0.00; //close
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
    else {
        double mid = static_cast<double>(last_index) - (static_cast<double>(longest) - 1.0) / 2.0;
        int mid_index = static_cast<int>(std::lround(mid));
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
    int run_start = -1;
    int run_len = 0;
    int shortest = std::numeric_limits<int>::max();
    int best_mid = -1;

    //loop to find the shortest sequence of far ranges (range = 1.0)
    for(int i=0; i<size; i++){
        if(sequence[i] > 0.5 ){
            if (run_len == 0) {
                run_start = i;
            }
            run_len++;
        } else {
            if (run_len >= index_width && run_len < shortest) {
                shortest = run_len;
                best_mid = run_start + (run_len - 1) / 2;
            }
            run_len = 0;
        }
    }
    // handle run that reaches the end
    if (run_len >= index_width && run_len < shortest) {
        shortest = run_len;
        best_mid = run_start + (run_len - 1) / 2;
    }
    return best_mid;
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


//given the middle index find the lowest range surrounding it 
float minRangeFromIndex(const std::vector<float>& ranges, const int mid_index) {
  // check all the parameters passed in are correct
  if (ranges.empty() || mid_index < 0 || mid_index >= static_cast<int>(ranges.size())) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  int start = std::max(0, mid_index - 5);
  int end = std::min(static_cast<int>(ranges.size()) - 1, mid_index + 5);
  float min_r = std::numeric_limits<float>::infinity();
  bool saw_finite = false;
  bool saw_nan = false;
  
  for (int i = start; i <= end; i++) {
    const float r = ranges[i];
    if (std::isnan(r)) {
      saw_nan = true;
      continue;
    }
    if (std::isinf(r)) {
      continue;
    }
    saw_finite = true;
    min_r = std::min(min_r, r);
  }
  if (saw_finite) {
    return min_r;
  }
  if (saw_nan) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return std::numeric_limits<float>::infinity();
}


//Given an index from laser scan, returns yaw with respect to front of robot
float LasertoFrontTF(const int mid_index, const float increment){
    float angle = rad2deg(increment*mid_index);
    if (angle <= 90){
        angle -= 90; 
    }
    else if(angle <= 270){
        angle -= 90;
    }
    else{
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


class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")

    {
        // Initialize publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        // LiDAR scan subscriber
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));

        // Bumper hazard detection subscriber
        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));

        // Turtlebot odometry subscriber
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
        start_yaw_ = 0.0;
        start_pos_x_ = 0.0;
        start_pos_y_ = 0.0;
        target_rotation_ = 0.0;
        target_move_ = 0.0;
        turning_ = false;
        moving_ = false;

        // Initialize LiDAR variables for navigation
        nLasers_ = 0;
        bestAngle_far_ = 0.0;
        bestClearance_far_ = 0.0;
        haveScan_far_ = false;
        bestAngle_close_ = 0.0;
        bestClearance_close_ = 0.0;
        haveScan_close_ = false;

        // Initialize LiDAR variables for obstacle avoidance
        min_front_dist_ = std::numeric_limits<float>::infinity();
        min_left_dist_  = std::numeric_limits<float>::infinity();
        min_right_dist_ = std::numeric_limits<float>::infinity();
        
        // Initialize Sequence
        far_sequence_ = true;
        midpoint_ = -1;
        turn_count_ = 0;

        // Initialize bumper states
        bumpers_["bump_front_left"]=false;
        bumpers_["bump_front_center"]=false;
        bumpers_["bump_front_right"]=false;
        bumpers_["bump_left"]=false;
        bumpers_["bump_right"]=false;
        
        // Log start message
        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }


private:
    // Helper function to calculate minimum distance within a field-of-view of the LIDAR
    void computeMinFOVDist(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        uint32_t center_idx,
        uint32_t fov_half_beams,
        float &min_dist_out)
    {
        // Set starting min dist to high value to be overwritten on first iteration
        min_dist_out = std::numeric_limits<float>::infinity();

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
            }
        }
    }

    void computeBestFromMidpoint(
        int midpoint,
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        float &angle_out,
        float &clearance_out,
        bool &have_out)
    {
        if (midpoint < 0 || midpoint >= static_cast<int>(scan->ranges.size())) {
            have_out = false;
            return;
        }

        angle_out = LasertoFrontTF(midpoint, scan->angle_increment);
        clearance_out = minRangeFromIndex(laserRange_, midpoint);

        if (std::isnan(clearance_out) || clearance_out <= 0.0f) {
            have_out = false;
            return;
        }
        if (std::isinf(clearance_out)) {
            clearance_out = scan->range_max;
        }
        if (!std::isfinite(clearance_out) || clearance_out <= 0.0f) {
            have_out = false;
            return;
        }

        have_out = true;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {

        //determine number of lasers
        nLasers_= static_cast<int32_t>((scan->angle_max - scan->angle_min)/scan->angle_increment);

        // 1. OBSTACLE AVOIDANCE LASER CALLBACK SECTION:

        // Number of laser beams in each FOV (all FOVs equal in angular size)
        uint32_t fov_half_beams = FOV_HALF_WIDTH / scan->angle_increment;

        //RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset %d", nLasers_, desiredNLasers_);

        // Array indices for each FOV
        uint32_t front_idx = (FRONT_ANGLE - scan->angle_min) / scan->angle_increment;
        uint32_t left_idx  = (LEFT_ANGLE  - scan->angle_min) / scan->angle_increment;
        uint32_t right_idx = (RIGHT_ANGLE - scan->angle_min) / scan->angle_increment;

        // Calculate minimum distance for each LIDAR FOV (front, left, right)
        computeMinFOVDist(scan, front_idx, fov_half_beams, min_front_dist_);
        computeMinFOVDist(scan, left_idx, fov_half_beams, min_left_dist_);
        computeMinFOVDist(scan, right_idx, fov_half_beams, min_right_dist_);
        
        // 2. NAVIGATION LASER CALLBACK SECTION:
        laserRange_ = scan->ranges;

        //Threshold for most wide space based on maximum laser distance
        float threshold_wide = 0.8f * maxRange(laserRange_); //Play around
        if (!std::isfinite(threshold_wide) || threshold_wide <= 0.0f) {
            threshold_wide = scan->range_max;
        }
        if (!std::isfinite(threshold_wide) || threshold_wide <= 0.0f) {
            threshold_wide = 1.0f;
        }
        
        //Threshold for most narrow space based on maximum laser distance
        float threshold_narrow = 0.75f * maxRange(laserRange_); //Play around
        if (!std::isfinite(threshold_narrow) || threshold_narrow <= 0.0f) {
            threshold_narrow = scan->range_max;
        }
        if (!std::isfinite(threshold_narrow) || threshold_narrow <= 0.0f) {
            threshold_narrow = 1.0f;
        }

             
        wideSpace_.resize(laserRange_.size());
        narrowSpace_.resize(laserRange_.size());


        classifyRanges(laserRange_, wideSpace_, laserRange_.size(), threshold_wide);
        classifyRanges(laserRange_, narrowSpace_, laserRange_.size(), threshold_narrow);

        
        int midpoint_far = midSequenceFar(wideSpace_, laserRange_.size());
        int midpoint_close = midSequenceClose(narrowSpace_, laserRange_.size(), threshold_narrow, scan->angle_increment);

        computeBestFromMidpoint(midpoint_far, scan, bestAngle_far_, bestClearance_far_, haveScan_far_);
        computeBestFromMidpoint(midpoint_close, scan, bestAngle_close_, bestClearance_close_, haveScan_close_);

        if (haveScan_far_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Far space: angle %f deg, range %f",
                                 rad2deg(bestAngle_far_), bestClearance_far_);
        }
        if (haveScan_close_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Narrow space: angle %f deg, range %f",
                                 rad2deg(bestAngle_close_), bestClearance_close_);
        }

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

        const float max_turn = deg2rad(120.0f);

        auto choose_turn_angle = [&](float &angle_out, const char* &mode_label) -> bool {
            auto within_turn_limit = [&](float angle) {
                return std::abs(angle) <= max_turn;
            };
            if (far_sequence_) {
                if (haveScan_far_ && within_turn_limit(bestAngle_far_)) {
                    angle_out = bestAngle_far_;
                    mode_label = "wide";
                    return true;
                }
                if (haveScan_close_ && within_turn_limit(bestAngle_close_)) {
                    angle_out = bestAngle_close_;
                    mode_label = "narrow (fallback)";
                    return true;
                }
            } else {
                if (haveScan_close_ && within_turn_limit(bestAngle_close_)) {
                    angle_out = bestAngle_close_;
                    mode_label = "narrow";
                    return true;
                }
                if (haveScan_far_ && within_turn_limit(bestAngle_far_)) {
                    angle_out = bestAngle_far_;
                    mode_label = "wide (fallback)";
                    return true;
                }
            }
            return false;
        };

        auto log_turn_choice_state = [&](const char* context) {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] turn_count=%d far_sequence=%s | far(have=%s, angle=%.2f, range=%.2f) "
                        "close(have=%s, angle=%.2f, range=%.2f)",
                        context,
                        turn_count_,
                        far_sequence_ ? "true" : "false",
                        haveScan_far_ ? "true" : "false",
                        rad2deg(bestAngle_far_),
                        bestClearance_far_,
                        haveScan_close_ ? "true" : "false",
                        rad2deg(bestAngle_close_),
                        bestClearance_close_);
        };

        // Log position, orientation, and min laser distances
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Position: (%.2f, %.2f), Orientation: %f rad or %f deg, Minimum laser distance in front, left, and right: (%.2f, %.2f, %.2f)", pos_x_, pos_y_, yaw_, rad2deg(yaw_), min_front_dist_, min_left_dist_, min_right_dist_);
        
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
                // Reached target distance, rotate 90deg and then keep moving
                RCLCPP_INFO(this->get_logger(), "Reached 0.15m backup, start turning");
                moving_ = false;

                // Capture starting yaw to compare against current yaw
                start_yaw_ = yaw_;

                // Update turn cycle before choosing
                turn_count_++;
                far_sequence_ = (turn_count_ % 5 == 1);
                log_turn_choice_state("post-backup");
                float chosen_angle = 0.0f;
                const char* mode_label = "unknown";
                if (!choose_turn_angle(chosen_angle, mode_label)) {
                    RCLCPP_WARN(this->get_logger(), "No valid scan to choose turn angle");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Choosing most %s space for rotation", mode_label);
                target_rotation_ = chosen_angle;

                // Start turning sequence
                turning_ = true;
                linear_ = 0.0;
                angular_ = (target_rotation_ > 0.0) ? 1.0 : -1.0;

                // turn_count_ already updated before choosing
            }
        }

        // Priority 1b: finish any in-progress 15 or 90 deg turn
        else if (turning_) {
            // Check how much the robot has rotated
            double angle_rotated = NormAngle(yaw_ - start_yaw_);

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

        // Priority 2: if bumper hit, back up 0.15 m
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

        // Priority 3: if obstacle to the left or right, start a 15 deg turn away from obstacle
        else if (!any_bumper_pressed && (min_left_dist_ < 0.2 || min_right_dist_ < 0.2)) {
            // Capture starting yaw (heading)
            start_yaw_ = yaw_;

            // If obstacle to the left within 0.4m, turn right 15deg
            if (min_left_dist_ < 0.2) {
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

        // Priority 4: if obstacle in front, decide which way to go based on lidar data (skewed to right)
        else if (!any_bumper_pressed && min_front_dist_ < 0.3) {
            // Capture starting yaw (heading)
            start_yaw_ = yaw_;

            // Update turn cycle before choosing
            turn_count_++;
            far_sequence_ = (turn_count_ % 5 == 1);
            log_turn_choice_state("front-obstacle");
            float chosen_angle = 0.0f;
            const char* mode_label = "unknown";
            if (!choose_turn_angle(chosen_angle, mode_label)) {
                RCLCPP_WARN(this->get_logger(), "No valid scan to choose turn angle");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Choosing most %s space for rotation", mode_label);
            target_rotation_ = chosen_angle;

            // Start turning sequence
            turning_ = true;
            linear_ = 0.0;
            angular_ = (target_rotation_ > 0.0) ? 1.0 : -1.0;

            // turn_count_ already updated before choosing
        }

        // Priority 5: move forward if clear (slow down when near walls)
        else if (!any_bumper_pressed && min_front_dist_ >= 0.3) {
            angular_ = 0.0;
            if (min_front_dist_ <= 0.4 || min_left_dist_ <= 0.4 || min_right_dist_ <= 0.4) {
                linear_ = 0.1;
            } else {
                linear_ = 0.25;
            }
        }

        // Fallback for errors: stop moving robot
        else {
            angular_ = 0.0;
            linear_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Else statement shutdown activated\n\n\n\n");
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
    std::vector<float> laserRange_; 
    std::vector<float> wideSpace_;
    std::vector<float> narrowSpace_;

    float bestAngle_far_;
    float bestClearance_far_;
    bool haveScan_far_;
    float bestAngle_close_;
    float bestClearance_close_;
    bool haveScan_close_;
    double start_yaw_;
    double start_pos_x_;
    double start_pos_y_;
    double target_rotation_;
    double target_move_;
    bool turning_;
    bool moving_;
    bool far_sequence_;
    int midpoint_;
    int turn_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
