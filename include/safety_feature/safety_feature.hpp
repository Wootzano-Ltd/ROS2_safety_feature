#ifndef SAFETY_FEATURE__SAFETY_FEATURE_HPP_
#define SAFETY_FEATURE__SAFETY_FEATURE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "moveit/move_group_interface/move_group_interface.h"

class SafetyFeature
{
public:
  SafetyFeature(const rclcpp::Node::SharedPtr& node_ptr);

private:
  void joint_torque_callback(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_states, const sensor_msgs::msg::JointState::ConstSharedPtr &required_torque);
  void digital_output_callback(const std_msgs::msg::Bool &msg);
  void analog_output_callback(const std_msgs::msg::Int64 &msg);
  void stop_or_resume_robot(bool stop);
  void stop_robot();
  void resume_robot();

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::SubscriptionOptions sub_options_;

  message_filters::Subscriber<sensor_msgs::msg::JointState> joint_state_subscription_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> required_torque_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr digital_output_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr analog_output_subscription_;

  const std::shared_ptr<rclcpp::Node> node_ptr_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  int analog_threshold_;
  std::vector<double> torque_diff_threshold_;
  bool is_stop_triggered_;
  std::mutex is_stop_triggered_mutex_;
};

#endif // SAFETY_FEATURE