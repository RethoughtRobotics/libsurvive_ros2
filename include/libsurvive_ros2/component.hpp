// Copyright 2022 Andrew Symington
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef LIBSURVIVE_ROS2__COMPONENT_HPP_
#define LIBSURVIVE_ROS2__COMPONENT_HPP_

#define SURVIVE_ENABLE_FULL_API

// C system
#include <os_generic.h>

// C++ system
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Other
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "libsurvive/survive_api.h"
#include "libsurvive/survive.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace libsurvive_ros2
{

  class Component : public rclcpp::Node
  {
  public:
    explicit Component(const rclcpp::NodeOptions &options);
    virtual ~Component();
    rclcpp::Time get_ros_time(const std::string &str, FLT timecode);
    void publish_imu(const sensor_msgs::msg::Imu &msg);
    void publish_velocity(const geometry_msgs::msg::TwistStamped &msg);

  private:
    // A controller assigned to a hand (config/vive_devices.yaml). Pressing A saves its position
    // and heading as a gravity-levelled static anchor frame and resets the target to it. While
    // the trackpad is
    // touched (the clutch), the target follows the controller's motion in the anchor frame;
    // otherwise it holds still. Re-engaging continues from where the target stopped.
    struct Hand
    {
      std::string serial;
      std::string anchor_frame;
      std::string target_frame;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr trigger_publisher;  // 0 released, 1 pulled
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr b_button_publisher;  // on each B press
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr thumbstick_publisher;  // direction, on change
      bool a_pressed = false;
      bool b_pressed = false;
      bool trackpad_touched = false;
      std::string thumbstick;  // direction held: north, south, east, west, or empty
      bool has_anchor = false;
      bool clutched = false;
      tf2::Transform grip;              // grip point in the controller frame (identity rotation)
      tf2::Transform anchor;            // anchor in tracking frame
      tf2::Transform target;            // target in anchor frame
      tf2::Transform clutch_controller; // controller in anchor frame when clutch engaged
      tf2::Transform clutch_target;     // target when clutch engaged
    };
    Hand *find_hand(const std::string &serial);
    void set_anchor(Hand &hand, const SurviveSimpleObject *obj);
    void update_target(Hand &hand, const SurvivePose &pose, const rclcpp::Time &stamp);

    void work();
    std::vector<Hand> hands_;
    SurviveSimpleContext *actx_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr cfg_publisher_;
    std::thread worker_thread_;
    rclcpp::Time last_base_station_update_;
    std::string tracking_frame_;
    double lighthouse_rate_;
    double thumbstick_edge_;
  };

} // namespace libsurvive_ros2

#endif // LIBSURVIVE_ROS2__COMPONENT_HPP_
