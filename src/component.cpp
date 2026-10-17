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

#include "libsurvive_ros2/component.hpp"

// tf2 headers must precede C++ system headers for the package linter.
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

// C++ system
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// Other
#include "rclcpp_components/register_node_macro.hpp"


// Scale factor to move from G to m/s^2.
constexpr double SI_GRAVITY = 9.80665;

// We can only ever load one version of the driver, so we store a pointer to the instance of the
// driver here, so the IMU callback can push data to it.
libsurvive_ros2::Component * _singleton = nullptr;

static void imu_func(
  SurviveObject * so, int mask, const FLT * accelgyromag, uint32_t rawtime, int id)
{
  if (_singleton) {
    survive_default_imu_process(so, mask, accelgyromag, rawtime, id);
    FLT timecode = SurviveSensorActivations_runtime(
      &so->activations, so->activations.last_imu) / FLT(1e6);
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = std::string(so->serial_number) + "_imu";
    imu_msg.header.stamp = _singleton->get_ros_time("inertial", timecode);
    imu_msg.angular_velocity.x = accelgyromag[3];
    imu_msg.angular_velocity.y = accelgyromag[4];
    imu_msg.angular_velocity.z = accelgyromag[5];
    imu_msg.linear_acceleration.x = accelgyromag[0] * SI_GRAVITY;
    imu_msg.linear_acceleration.y = accelgyromag[1] * SI_GRAVITY;
    imu_msg.linear_acceleration.z = accelgyromag[2] * SI_GRAVITY;
    _singleton->publish_imu(imu_msg);
  }
}

static void ros_from_pose(
  geometry_msgs::msg::Transform * const tx, const SurvivePose & pose)
{
  tx->translation.x = pose.Pos[0];
  tx->translation.y = pose.Pos[1];
  tx->translation.z = pose.Pos[2];
  tx->rotation.w = pose.Rot[0];
  tx->rotation.x = pose.Rot[1];
  tx->rotation.y = pose.Rot[2];
  tx->rotation.z = pose.Rot[3];
}

static tf2::Quaternion body_from_world_quaternion(const SurvivePose & pose)
{
  tf2::Quaternion q_world_from_body(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]);
  q_world_from_body.normalize();
  return q_world_from_body.inverse();
}

// libsurvive reports an all-zero rotation, or a non-finite position, for objects it has not solved
// (e.g. an HMD out of view, or controllers while the lighthouse solve has not converged); tf2
// rejects those, and an anchor or target built on them would be meaningless, so they are dropped.
static bool has_valid_pose(const SurvivePose & pose)
{
  const double norm_sq = pose.Rot[0] * pose.Rot[0] + pose.Rot[1] * pose.Rot[1] +
    pose.Rot[2] * pose.Rot[2] + pose.Rot[3] * pose.Rot[3];
  return norm_sq > 1e-6 && std::isfinite(pose.Pos[0]) && std::isfinite(pose.Pos[1]) &&
         std::isfinite(pose.Pos[2]);
}

static tf2::Transform tf2_from_pose(const SurvivePose & pose)
{
  return tf2::Transform(
    tf2::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]).normalized(),
    tf2::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
}

// Make an anchor whose z axis is tracking-world up and whose x axis is the horizontal
// projection of the controller's pointing (+z) axis. This preserves the operator's heading at
// the A press while rejecting controller pitch and roll, which would otherwise mix horizontal
// hand motion into vertical target motion.
static bool gravity_levelled_anchor(const SurvivePose & pose, tf2::Transform * anchor)
{
  const tf2::Transform controller = tf2_from_pose(pose);
  const tf2::Vector3 up(0.0, 0.0, 1.0);
  tf2::Vector3 forward = controller.getBasis() * tf2::Vector3(0.0, 0.0, 1.0);
  forward -= up * forward.dot(up);
  if (forward.length2() < 1e-6) {
    return false;
  }
  forward.normalize();
  const tf2::Vector3 left = up.cross(forward);
  const tf2::Matrix3x3 basis(
    forward.x(), left.x(), up.x(),
    forward.y(), left.y(), up.y(),
    forward.z(), left.z(), up.z());
  tf2::Quaternion rotation;
  basis.getRotation(rotation);
  *anchor = tf2::Transform(rotation.normalized(), controller.getOrigin());
  return true;
}

namespace libsurvive_ros2
{

Component::Component(const rclcpp::NodeOptions & options)
: Node("libsurvive_ros2", options),
  actx_(nullptr),
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
  tf_static_broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this))
{
  // Store the instance globally to be used by a C callback.
  _singleton = this;

  // Global parameters
  this->declare_parameter("tracking_frame", "libsurvive_frame");
  this->get_parameter("tracking_frame", tracking_frame_);
  this->declare_parameter("lighthouse_rate", 4.0);
  this->get_parameter("lighthouse_rate", lighthouse_rate_);
  thumbstick_edge_ = this->declare_parameter("thumbstick_edge", 0.9);

  // Fixed placement of the tracking frame under the robot base (see config/vive_devices.yaml).
  const std::string robot_frame = this->declare_parameter<std::string>("robot_frame", "");
  const std::vector<double> xyz = this->declare_parameter<std::vector<double>>(
    "tracking_frame_xyz", {0.0, 0.0, 0.0});
  if (!robot_frame.empty() && xyz.size() == 3) {
    geometry_msgs::msg::TransformStamped placement;
    placement.header.stamp = this->get_clock()->now();
    placement.header.frame_id = robot_frame;
    placement.child_frame_id = tracking_frame_;
    placement.transform.translation.x = xyz[0];
    placement.transform.translation.y = xyz[1];
    placement.transform.translation.z = xyz[2];
    placement.transform.rotation.w = 1.0;
    tf_static_broadcaster_->sendTransform(placement);
  }

  // Setup topic for IMU.
  std::string imu_topic;
  this->declare_parameter("imu_topic", "imu");
  this->get_parameter("imu_topic", imu_topic);
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

  std::string velocity_topic;
  this->declare_parameter("velocity_topic", "velocity");
  this->get_parameter("velocity_topic", velocity_topic);
  velocity_publisher_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>(velocity_topic, 10);

  // Setup topic for joystick.
  std::string joy_topic;
  this->declare_parameter("joy_topic", "joy");
  this->get_parameter("joy_topic", joy_topic);
  joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic, 10);

  // Setup topic for configuration.
  std::string cfg_topic;
  this->declare_parameter("cfg_topic", "cfg");
  this->get_parameter("cfg_topic", cfg_topic);
  cfg_publisher_ = this->create_publisher<diagnostic_msgs::msg::KeyValue>(cfg_topic, 10);

  // Setup hands for anchoring (see config/vive_devices.yaml).
  for (const std::string side : {"left", "right"}) {
    Hand hand;
    hand.serial = this->declare_parameter<std::string>(side + "_serial", "");
    hand.anchor_frame = this->declare_parameter<std::string>(
      side + "_anchor_frame", side + "_anchor");
    hand.target_frame = this->declare_parameter<std::string>(
      side + "_target_frame", side + "_target");
    const std::string target_topic = this->declare_parameter<std::string>(
      side + "_target_topic", "/vive/" + side + "/target");
    const std::string trigger_topic = this->declare_parameter<std::string>(
      side + "_trigger_topic", "/vive/" + side + "/trigger");
    const std::string b_button_topic = this->declare_parameter<std::string>(
      side + "_b_button_topic", "/vive/" + side + "/b_button");
    const std::string thumbstick_topic = this->declare_parameter<std::string>(
      side + "_thumbstick_topic", "/vive/" + side + "/thumbstick");
    const std::vector<double> grip = this->declare_parameter<std::vector<double>>(
      side + "_grip_offset", {0.0, 0.0, 0.0});
    hand.grip.setIdentity();
    hand.grip.setOrigin(tf2::Vector3(grip.at(0), grip.at(1), grip.at(2)));
    if (hand.serial.empty()) {
      continue;
    }
    hand.target_publisher =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(target_topic, 10);
    hand.trigger_publisher = this->create_publisher<std_msgs::msg::Float32>(trigger_topic, 10);
    hand.b_button_publisher = this->create_publisher<std_msgs::msg::Empty>(b_button_topic, 10);
    // Latched, so a subscriber that joins late still gets the current direction.
    hand.thumbstick_publisher = this->create_publisher<std_msgs::msg::String>(
      thumbstick_topic, rclcpp::QoS(1).transient_local());
    RCLCPP_INFO(
      this->get_logger(), "%s hand: press A on %s to set %s, touch trackpad to move %s",
      side.c_str(), hand.serial.c_str(), hand.anchor_frame.c_str(), hand.target_frame.c_str());
    hands_.push_back(hand);
  }

  // Setup driver parameters.
  std::string driver_args;
  this->declare_parameter("driver_args", "--force-recalibrate 1");
  this->get_parameter("driver_args", driver_args);
  // libsurvive parses these like main()'s argv, skipping argv[0] (the program name). `tokens` owns
  // the strings: `args` only points into it, so it must outlive survive_simple_init.
  std::vector<std::string> tokens{"libsurvive_ros2"};
  std::stringstream driver_ss(driver_args);
  std::string token;
  while (getline(driver_ss, token, ' ')) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  std::vector<const char *> args;
  for (const std::string & t : tokens) {
    args.push_back(t.c_str());
  }

  // Try and initialize survive with the arguments supplied.
  actx_ = survive_simple_init(args.size(), const_cast<char **>(args.data()));
  if (actx_ == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "Could not initialize the libsurvive context");
    return;
  }

  // Setup callback for reading IMU data.
  SurviveContext * ctx = survive_simple_get_ctx(actx_);
  survive_install_imu_fn(ctx, imu_func);

  // Initialize the survive thread.
  survive_simple_start_thread(actx_);

  // Start the work thread
  worker_thread_ = std::thread(&Component::work, this);
}

Component::~Component()
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up.");
  worker_thread_.join();

  RCLCPP_INFO(this->get_logger(), "Shutting down libsurvive driver");
  if (actx_) {
    survive_simple_close(actx_);
  }

  RCLCPP_INFO(this->get_logger(), "Clearing singleton instance");
  _singleton = nullptr;
}

rclcpp::Time Component::get_ros_time(const std::string & /*str*/, FLT timecode)
{
  return rclcpp::Time() + rclcpp::Duration(std::chrono::duration<double>(timecode));
}

void Component::publish_imu(const sensor_msgs::msg::Imu & msg)
{
  if (imu_publisher_) {
    imu_publisher_->publish(msg);
  }
}

void Component::publish_velocity(const geometry_msgs::msg::TwistStamped & msg)
{
  if (velocity_publisher_) {
    velocity_publisher_->publish(msg);
  }
}

Component::Hand * Component::find_hand(const std::string & serial)
{
  for (Hand & hand : hands_) {
    if (hand.serial == serial) {
      return &hand;
    }
  }
  return nullptr;
}

void Component::set_anchor(Hand & hand, const SurviveSimpleObject * obj)
{
  SurvivePose pose = {};
  const auto timecode = survive_simple_object_get_latest_pose(obj, &pose);
  if (timecode <= 0 || !has_valid_pose(pose)) {
    RCLCPP_WARN(
      this->get_logger(), "A pressed on %s but it is not tracked (no valid pose yet)", hand.serial.c_str());
    return;
  }
  if (!gravity_levelled_anchor(pose, &hand.anchor)) {
    RCLCPP_WARN(
      this->get_logger(), "A pressed on %s while its pointing axis is vertical",
      hand.serial.c_str());
    return;
  }
  hand.target.setIdentity();
  hand.clutched = false;  // re-engages against the new anchor on the next pose
  hand.has_anchor = true;

  // The C++ static broadcaster replaces an existing child frame, so re-anchoring works.
  geometry_msgs::msg::TransformStamped anchor_msg;
  anchor_msg.header.stamp = this->get_ros_time("anchor", timecode);
  anchor_msg.header.frame_id = tracking_frame_;
  anchor_msg.child_frame_id = hand.anchor_frame;
  const tf2::Vector3 & anchor_position = hand.anchor.getOrigin();
  const tf2::Quaternion & anchor_rotation = hand.anchor.getRotation();
  anchor_msg.transform.translation.x = anchor_position.x();
  anchor_msg.transform.translation.y = anchor_position.y();
  anchor_msg.transform.translation.z = anchor_position.z();
  anchor_msg.transform.rotation.x = anchor_rotation.x();
  anchor_msg.transform.rotation.y = anchor_rotation.y();
  anchor_msg.transform.rotation.z = anchor_rotation.z();
  anchor_msg.transform.rotation.w = anchor_rotation.w();
  tf_static_broadcaster_->sendTransform(anchor_msg);
  RCLCPP_INFO(
    this->get_logger(), "%s gravity-levelled at (%+.3f, %+.3f, %+.3f)",
    hand.anchor_frame.c_str(), pose.Pos[0], pose.Pos[1], pose.Pos[2]);
}

void Component::update_target(Hand & hand, const SurvivePose & pose, const rclcpp::Time & stamp)
{
  if (!hand.has_anchor) {
    return;
  }
  // Track the grip point rather than the tracking origin, so twisting the controller about
  // where it is held turns the target in place instead of swinging it through an arc.
  const tf2::Transform controller = hand.anchor.inverse() * tf2_from_pose(pose) * hand.grip;
  if (hand.trackpad_touched && !hand.clutched) {
    hand.clutch_controller = controller;
    hand.clutch_target = hand.target;
  }
  hand.clutched = hand.trackpad_touched;

  // Apply only the motion made since the clutch engaged, in anchor-frame axes, so
  // releasing freezes the target and re-engaging never jumps.
  if (hand.clutched) {
    hand.target.setOrigin(
      hand.clutch_target.getOrigin() +
      (controller.getOrigin() - hand.clutch_controller.getOrigin()));
    hand.target.setRotation(
      (controller.getRotation() * hand.clutch_controller.getRotation().inverse() *
      hand.clutch_target.getRotation()).normalized());
  }

  const tf2::Vector3 & p = hand.target.getOrigin();
  const tf2::Quaternion q = hand.target.getRotation();
  geometry_msgs::msg::TransformStamped target_tf;
  target_tf.header.stamp = stamp;
  target_tf.header.frame_id = hand.anchor_frame;
  target_tf.child_frame_id = hand.target_frame;
  target_tf.transform.translation.x = p.x();
  target_tf.transform.translation.y = p.y();
  target_tf.transform.translation.z = p.z();
  target_tf.transform.rotation.x = q.x();
  target_tf.transform.rotation.y = q.y();
  target_tf.transform.rotation.z = q.z();
  target_tf.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(target_tf);

  geometry_msgs::msg::PoseStamped msg;
  msg.header = target_tf.header;
  msg.pose.position.x = p.x();
  msg.pose.position.y = p.y();
  msg.pose.position.z = p.z();
  msg.pose.orientation = target_tf.transform.rotation;
  hand.target_publisher->publish(msg);
}

void Component::work()
{
  RCLCPP_INFO(this->get_logger(), "Start listening for events..");

  // Poll for events.
  struct SurviveSimpleEvent event = {};
  while (survive_simple_wait_for_event(
      actx_,
      &event) != SurviveSimpleEventType_Shutdown && rclcpp::ok())
  {
    // Business logic depends on the event type
    switch (event.event_type) {
      // TYPE: Pose update (limit to non-lighthouses only)
      case SurviveSimpleEventType_PoseUpdateEvent: {
          const struct SurviveSimplePoseUpdatedEvent * pose_event =
            survive_simple_get_pose_updated_event(&event);
          if (survive_simple_object_get_type(pose_event->object) !=
            SurviveSimpleObject_LIGHTHOUSE)
          {
            SurvivePose pose = {};
            auto timecode = survive_simple_object_get_latest_pose(pose_event->object, &pose);
            if (timecode > 0 && has_valid_pose(pose)) {
              const std::string serial = survive_simple_serial_number(pose_event->object);
              geometry_msgs::msg::TransformStamped pose_msg;
              pose_msg.header.stamp = this->get_ros_time("tracker", timecode);
              pose_msg.header.frame_id = tracking_frame_;
              pose_msg.child_frame_id = serial;
              ros_from_pose(&pose_msg.transform, pose);
              tf_broadcaster_->sendTransform(pose_msg);
              if (Hand * hand = find_hand(serial)) {
                update_target(*hand, pose, pose_msg.header.stamp);
              }

              SurviveVelocity velocity = {};
              const auto velocity_timecode =
                survive_simple_object_get_latest_velocity(pose_event->object, &velocity);
              if (velocity_timecode > 0) {
                const tf2::Quaternion q_body_from_world = body_from_world_quaternion(pose);
                const tf2::Vector3 linear_world(
                  velocity.Pos[0], velocity.Pos[1], velocity.Pos[2]);
                const tf2::Vector3 angular_world(
                  velocity.AxisAngleRot[0], velocity.AxisAngleRot[1], velocity.AxisAngleRot[2]);
                const tf2::Vector3 linear_body = tf2::quatRotate(q_body_from_world, linear_world);
                const tf2::Vector3 angular_body = tf2::quatRotate(q_body_from_world, angular_world);

                geometry_msgs::msg::TwistStamped velocity_msg;
                velocity_msg.header.stamp = this->get_ros_time("velocity", velocity_timecode);
                velocity_msg.header.frame_id = serial;
                velocity_msg.twist.linear.x = linear_body.x();
                velocity_msg.twist.linear.y = linear_body.y();
                velocity_msg.twist.linear.z = linear_body.z();
                velocity_msg.twist.angular.x = angular_body.x();
                velocity_msg.twist.angular.y = angular_body.y();
                velocity_msg.twist.angular.z = angular_body.z();
                publish_velocity(velocity_msg);
              }
            }
          }
          break;
        }

      // TYPE: Button update
      case SurviveSimpleEventType_ButtonEvent: {
          const struct SurviveSimpleButtonEvent * button_event = survive_simple_get_button_event(
            &event);
          auto obj = button_event->object;
          sensor_msgs::msg::Joy joy_msg;
          joy_msg.header.frame_id = survive_simple_serial_number(button_event->object);
          joy_msg.header.stamp = this->get_ros_time("button", button_event->time);
          joy_msg.axes.resize(SURVIVE_MAX_AXIS_COUNT * 2);
          joy_msg.buttons.resize(SURVIVE_BUTTON_MAX * 2);
          int64_t mask = survive_simple_object_get_button_mask(obj);
          mask |= (survive_simple_object_get_touch_mask(obj) << SURVIVE_BUTTON_MAX);
          for (int i = 0; i < SURVIVE_MAX_AXIS_COUNT * 2; i++) {
            joy_msg.axes[i] =
              static_cast<float>(survive_simple_object_get_input_axis(obj, (enum SurviveAxis)i));
          }
          for (int i = 0; i < mask && i < static_cast<int>(joy_msg.buttons.size()); i++) {
            joy_msg.buttons[i] = (mask >> i) & 1;
          }
          joy_publisher_->publish(joy_msg);

          if (Hand * hand = find_hand(joy_msg.header.frame_id)) {
            const bool pressed = (mask >> SURVIVE_BUTTON_A) & 1;
            if (pressed && !hand->a_pressed) {
              set_anchor(*hand, obj);
            }
            hand->a_pressed = pressed;
            const bool b_pressed = (mask >> SURVIVE_BUTTON_B) & 1;
            if (b_pressed && !hand->b_pressed) {
              hand->b_button_publisher->publish(std_msgs::msg::Empty());  // e.g. teleop: arm home
            }
            hand->b_pressed = b_pressed;
            hand->trackpad_touched =
              (mask >> (SURVIVE_BUTTON_MAX + SURVIVE_BUTTON_TRACKPAD)) & 1;
            // The thumbstick direction held (|axis| >= thumbstick_edge, the larger axis wins):
            // north +y, south -y, east +x, west -x, or empty.
            const float x = joy_msg.axes[SURVIVE_AXIS_JOYSTICK_X];
            const float y = joy_msg.axes[SURVIVE_AXIS_JOYSTICK_Y];
            std::string direction;
            if (std::abs(y) >= thumbstick_edge_ && std::abs(y) >= std::abs(x)) {
              direction = y > 0 ? "north" : "south";
            } else if (std::abs(x) >= thumbstick_edge_) {
              direction = x > 0 ? "east" : "west";
            }
            if (direction != hand->thumbstick) {
              hand->thumbstick = direction;
              std_msgs::msg::String thumbstick;
              thumbstick.data = direction;
              hand->thumbstick_publisher->publish(thumbstick);
            }
            // The analog trigger (0 released .. 1 pulled), e.g. for the gripper.
            std_msgs::msg::Float32 trigger;
            trigger.data = joy_msg.axes[SURVIVE_AXIS_TRIGGER];
            hand->trigger_publisher->publish(trigger);
          }
          break;
        }

      // TYPE: Configuration update
      case SurviveSimpleEventType_ConfigEvent: {
          const struct SurviveSimpleConfigEvent * config_event = survive_simple_get_config_event(
            &event);
          diagnostic_msgs::msg::KeyValue cfg_msg;
          cfg_msg.key = survive_simple_serial_number(config_event->object);
          cfg_msg.value = config_event->cfg;
          cfg_publisher_->publish(cfg_msg);
          break;
        }

      // TYPE: Device add event
      case SurviveSimpleEventType_DeviceAdded: {
          const struct SurviveSimpleObjectEvent * object_event = survive_simple_get_object_event(
            &event);
          RCLCPP_INFO(
            this->get_logger(), "A new device %s was added at time %lf",
            survive_simple_serial_number(object_event->object),
            this->get_ros_time("connect", object_event->time).seconds()
          );
          break;
        }

      // TYPE: no-op
      case SurviveSimpleEventType_None: {
          break;
        }

      // We should never get here.
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown event");
        break;
    }

    // Always update the base stations
    auto time_now = this->get_clock()->now();
    if (time_now.seconds() - last_base_station_update_.seconds() > lighthouse_rate_) {
      last_base_station_update_ = time_now;
      for (const SurviveSimpleObject * it = survive_simple_get_first_object(actx_); it != 0;
        it = survive_simple_get_next_object(actx_, it))
      {
        if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
          SurvivePose pose = {};
          auto timecode = survive_simple_object_get_latest_pose(it, &pose);
          if (timecode > 0 && has_valid_pose(pose)) {
            geometry_msgs::msg::TransformStamped pose_msg;
            pose_msg.header.stamp = this->get_ros_time("lighthouse", timecode);
            pose_msg.header.frame_id = tracking_frame_;
            pose_msg.child_frame_id = survive_simple_serial_number(it);
            ros_from_pose(&pose_msg.transform, pose);
            tf_static_broadcaster_->sendTransform(pose_msg);
          }
        }
      }
    }
  }
}

}  // namespace libsurvive_ros2

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(libsurvive_ros2::Component)
