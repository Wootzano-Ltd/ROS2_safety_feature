#include "safety_feature/safety_feature.hpp"

SafetyFeature::SafetyFeature(const rclcpp::Node::SharedPtr &node_ptr)
    : node_ptr_(node_ptr), move_group_(node_ptr, std::string(node_ptr_->get_parameter("planning_group").as_string())), is_stop_triggered_(false)
{
  cb_group_ = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  sub_options_.callback_group = cb_group_;

  joint_state_subscription_.subscribe(node_ptr_, "joint_states", rmw_qos_profile_sensor_data, sub_options_);
  required_torque_subscription_.subscribe(node_ptr_, "required_torque", rmw_qos_profile_sensor_data, sub_options_);
  message_filters::TimeSynchronizer<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> time_sync(joint_state_subscription_, required_torque_subscription_, 1);
  time_sync.registerCallback(std::bind(&SafetyFeature::joint_torque_callback, this, std::placeholders::_1, std::placeholders::_2));

  digital_output_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Bool>("digital_output", 1, std::bind(&SafetyFeature::digital_output_callback, this, std::placeholders::_1), sub_options_);
  analog_output_subscription_ = node_ptr_->create_subscription<std_msgs::msg::Int64>("analog_output", 1, std::bind(&SafetyFeature::analog_output_callback, this, std::placeholders::_1), sub_options_);

  analog_threshold_ = node_ptr_->get_parameter("analog_threshold").as_int();
  torque_diff_threshold_ = node_ptr_->get_parameter("torque_diff_threshold").as_double_array();

  RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Safety feature has been initialized !");
}

void SafetyFeature::joint_torque_callback(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_states, const sensor_msgs::msg::JointState::ConstSharedPtr &required_torque)
{
  if ((joint_states->effort.size() != required_torque->effort.size()) || joint_states->effort.size() != torque_diff_threshold_.size())
  {
    RCLCPP_ERROR_STREAM(node_ptr_->get_logger(), "Cannot calculate joint torque error ! Check the size of joint states, required torque and torque error threshold.");
    return;
  }

  bool stop = false;
  // check if there is any joint whose torque diff exceed threshold
  for (size_t i = 0; i < joint_states->effort.size(); ++i)
  {
    stop = stop || std::abs(joint_states->effort.at(i) - required_torque->effort.at(i)) > torque_diff_threshold_.at(i);
  }

  stop_or_resume_robot(stop);
}

void SafetyFeature::digital_output_callback(const std_msgs::msg::Bool &msg)
{
  stop_or_resume_robot(msg.data);
}

void SafetyFeature::analog_output_callback(const std_msgs::msg::Int64 &msg)
{
  stop_or_resume_robot(msg.data > analog_threshold_);
}

void SafetyFeature::stop_or_resume_robot(bool stop)
{
  if (stop)
  {
    const std::lock_guard<std::mutex> lock(is_stop_triggered_mutex_);
    if (!is_stop_triggered_)
    {
      is_stop_triggered_ = true;
      std::thread thread_stop(&SafetyFeature::stop_robot, this);
      thread_stop.detach();
    }
  }
  else
  {
    const std::lock_guard<std::mutex> lock(is_stop_triggered_mutex_);
    if (is_stop_triggered_)
    {
      resume_robot();
    }
  }
}

void SafetyFeature::stop_robot()
{
  RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "stop_robot is called !");
  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    // You may want to disable robot rather than using moveit.
    move_group_.stop();
    rate.sleep();
    const std::lock_guard<std::mutex> lock(is_stop_triggered_mutex_);
    if (!is_stop_triggered_)
      break;
  }
}

void SafetyFeature::resume_robot()
{
  RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "resume_robot is called !");
  is_stop_triggered_ = false;
}