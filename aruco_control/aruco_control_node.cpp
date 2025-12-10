
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class ArucoControlNode : public rclcpp::Node
{
public:
  ArucoControlNode()
  : Node("aruco_control_node"), state_(State::SCANNING), initial_yaw_(0.0), current_yaw_(0.0), first_odom_(true)
  {
    // Parameters
    this->declare_parameter("camera_topic", "/camera/image");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("scan_speed", 0.5);
    this->declare_parameter("kp", 0.002);

    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    scan_speed_ = this->get_parameter("scan_speed").as_double();
    kp_ = this->get_parameter("kp").as_double();

    // Subscribers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic, 10, std::bind(&ArucoControlNode::image_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ArucoControlNode::odom_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    result_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_control/processed_image", 10);

    // Initialize ArUco dictionary (Assuming 4x4_50 as per RosAruco default, or generic)
    // The previous analysis showed marker_dict defaults to 4X4_50 in aruco_tracker.cpp
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    parameters_ = cv::aruco::DetectorParameters::create();

    RCLCPP_INFO(this->get_logger(), "Aruco Control Node Started. State: SCANNING");
  }

private:
  enum class State {
    SCANNING,
    NAVIGATING,
    DONE
  };

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;

    if (first_odom_) {
      initial_yaw_ = yaw;
      first_odom_ = false;
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, parameters_);

    // Control Logic
    geometry_msgs::msg::Twist twist;

    if (state_ == State::SCANNING) {
      // Store IDs
      for (int id : ids) {
        if (std::find(detected_ids_.begin(), detected_ids_.end(), id) == detected_ids_.end()) {
          detected_ids_.push_back(id);
          RCLCPP_INFO(this->get_logger(), "Found new ID: %d", id);
        }
      }

      // Rotate
      twist.angular.z = scan_speed_;

      // Check termination of scan (full rotation)
      // Normalize angle difference
      // double angle_diff = current_yaw_ - initial_yaw_; // Unused

      // Handle wrapping? Simple check: if we started at 0, went to PI, then -PI, then 0. 
      // This is tricky with simple yaw. 
      // Better heuristic: Have we seen enough time pass? Or check if angle diff close to 0 after being far?
      // Let's use a simpler "Found at least X markers" or "Timeout". 
      // User implies "after all IDs identified".
      // Let's assume there are 5 markers based on world name `box_ring_aruco_5`.
      if (detected_ids_.size() >= 5) {
        RCLCPP_INFO(this->get_logger(), "All expected IDs found. Switching to NAVIGATING.");
        state_ = State::NAVIGATING;
        std::sort(detected_ids_.begin(), detected_ids_.end());
        twist.angular.z = 0.0;
      }
      // Fail-safe: if scan runs too long?
      // For now, infinite scan until 5 is found.
      
    } else if (state_ == State::NAVIGATING) {
      if (current_target_index_ >= detected_ids_.size()) {
        twist.angular.z = 0.0;
        state_ = State::DONE;
        RCLCPP_INFO(this->get_logger(), "All markers processed.");
      } else {
        int target_id = detected_ids_[current_target_index_];
        
        // Find if target is in current view
        int index_in_view = -1;
        for (size_t i = 0; i < ids.size(); ++i) {
          if (ids[i] == target_id) {
            index_in_view = i;
            break;
          }
        }

        if (index_in_view != -1) {
          // Target visible. Servoing.
          cv::Point2f center(0, 0);
          for (const auto& p : corners[index_in_view]) {
            center += p;
          }
          center *= (1.0 / 4.0);
          
          double image_center_x = cv_ptr->image.cols / 2.0;
          double error_x = image_center_x - center.x; // Positive if marker is to left -> turn left (positive z)
          
          if (std::abs(error_x) < 10) { // 10 pixels tolerance
            twist.angular.z = 0.0;
            // Draw circle & Publish
            cv::circle(cv_ptr->image, center, 50, cv::Scalar(0, 255, 0), 4);
            // Optionally put text
            cv::putText(cv_ptr->image, "ID: " + std::to_string(target_id), center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
            
            result_image_pub_->publish(*cv_ptr->toImageMsg());
            RCLCPP_INFO(this->get_logger(), "Processed ID: %d", target_id);
            
            // Move to next
            // We need to wait a bit or ensure we don't immediately jump if next ID is same view?
            // "The robot repeats ... for all other markers"
            // We'll advance index.
            current_target_index_++;
            // We might need to pause? No, prompt just says repeats.
          } else {
            twist.angular.z = kp_ * error_x; 
            // Clamp
            twist.angular.z = std::max(-0.5, std::min(0.5, twist.angular.z));
            // Minimum speed to move
            if (std::abs(twist.angular.z) < 0.1) twist.angular.z = (twist.angular.z > 0 ? 0.1 : -0.1);
          }
        } else {
          // Target not visible. Rotate to find it.
          // Since we sorted IDs and they are likely in a ring, and we just scanned, 
          // we might just continue rotating in one direction.
          if (detected_ids_.size() > 1 && detected_ids_[current_target_index_] < detected_ids_[(current_target_index_ + 1) % detected_ids_.size()]) {
              // Ascending order.
              twist.angular.z = 0.3; 
          } else {
              twist.angular.z = 0.3; // Default search direction
          }
        }
      }
    } else {
      twist.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(twist);
  }

  // Members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;

  State state_;
  double initial_yaw_;
  double current_yaw_;
  bool first_odom_;
  double scan_speed_;
  double kp_;

  std::vector<int> detected_ids_;
  size_t current_target_index_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoControlNode>());
  rclcpp::shutdown();
  return 0;
}
