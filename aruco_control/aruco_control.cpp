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
  : Node("aruco_control"), state_(State::SCANNING), initial_yaw_(0.0), current_yaw_(0.0), first_odom_(true)
  {
    // Parameters
    this->declare_parameter("camera_topic", "/camera/image");
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("scan_speed", 0.4);
    this->declare_parameter("kp_ang", 0.002);
    this->declare_parameter("kp_lin", 0.00005);
    this->declare_parameter("target_area", 15000.0);
    this->declare_parameter("linear_limit", 0.5);

    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    scan_speed_ = this->get_parameter("scan_speed").as_double();
    kp_ang_ = this->get_parameter("kp_ang").as_double();
    kp_lin_ = this->get_parameter("kp_lin").as_double();
    target_area_ = this->get_parameter("target_area").as_double();
    linear_limit_ = this->get_parameter("linear_limit").as_double();


    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_topic, 10, std::bind(&ArucoControlNode::image_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&ArucoControlNode::odom_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    result_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco_control/processed_image", 10);

    // Initialize ArUco dictionary 
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
    process_image(cv_ptr);
  }
  
  void process_image(cv_bridge::CvImagePtr cv_ptr)
  {
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

      // Check termination of scan
      if (detected_ids_.size() >= 5) { // Assuming 5 markers
        RCLCPP_INFO(this->get_logger(), "All expected IDs found. Switching to NAVIGATING.");
        state_ = State::NAVIGATING;
        std::sort(detected_ids_.begin(), detected_ids_.end());    
        // Stop rotation
        twist.angular.z = 0.0;
      }


    } else if (state_ == State::NAVIGATING) {
      if (current_target_index_ >= detected_ids_.size()) {
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
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
          // Target visible. visual servoing.
          
          // 1. Angular Control (Centering)
          cv::Point2f center(0, 0);
          for (const auto& p : corners[index_in_view]) {
            center += p;
          }
          center *= (1.0 / 4.0); // Average of 4 corners
          
          double image_center_x = cv_ptr->image.cols / 2.0;
          double error_ang = image_center_x - center.x; // Positive if marker is to LEFT -> turn Left (+)
          
          // 2. Linear Control (Approach based on area)
          double area = cv::contourArea(corners[index_in_view]);
          double error_lin = target_area_ - area; // Positive if too small -> move forward (+)

          bool aligned_ang = std::abs(error_ang) < 20; // 20 pixels angular tolerance
          bool aligned_lin = std::abs(error_lin) < (target_area_ * 0.1); // 10% area tolerance

          if (aligned_ang && aligned_lin) {
             // Goal Reached for this marker
             twist.angular.z = 0.0;
             twist.linear.x = 0.0;
             
             // Draw circle & Publish
             cv::circle(cv_ptr->image, center, 50, cv::Scalar(0, 255, 0), 4);
             cv::putText(cv_ptr->image, "ID: " + std::to_string(target_id), center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
             result_image_pub_->publish(*cv_ptr->toImageMsg());
             cv::imshow("Processed Image", cv_ptr->image);
             cv::waitKey(1);
             
             RCLCPP_INFO(this->get_logger(), "Reached ID: %d. Moving to next.", target_id);
             
             // Move to next
             current_target_index_++;
             
          } else {
             // Control Law
             
             // Angular
             twist.angular.z = kp_ang_ * error_ang;
             twist.angular.z = std::max(-0.5, std::min(0.5, twist.angular.z)); // Limit angular speed
             
             // Linear - Only move forward if somewhat centered to avoid losing it
             if (std::abs(error_ang) < 100) {
                 twist.linear.x = kp_lin_ * error_lin;
                 twist.linear.x = std::max(-linear_limit_, std::min(linear_limit_, twist.linear.x));
             } else {
                 twist.linear.x = 0.0;
             }
          }

        } else {
          // Target not visible. Rotate to find it.
          // Simple search: Rotate in one direction
          twist.angular.z = 0.3; 
          twist.linear.x = 0.0;
        }
      }
    } else {
      twist.angular.z = 0.0;
      twist.linear.x = 0.0;
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
  double kp_ang_;
  double kp_lin_;
  double target_area_;
  double linear_limit_;

  std::vector<int> detected_ids_;
  size_t current_target_index_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
