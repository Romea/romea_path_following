//std
#include <functional>
#include <type_traits>

//romea
// #include <math/Algorithm.hpp>
// #include <ros/CommandFactory.hpp>
// #include <ros/ObserverFactory.hpp>
// #include <ros/params/RosParam.hpp>
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_common/math/Algorithm.hpp>
#include <romea_mobile_base_utils/params/command_interface_parameters.hpp>

#include "romea_path_following/path_following.hpp"

#ifdef ROMEA_CONTROL_PRIVATE
#include <ros/PrivateCommandFactory.hpp>
#include <ros/PrivateObserverFactory.hpp>
#endif

namespace
{
const int XBOX_Y_BUTTON = 3;
const int XBOX_X_BUTTON = 2;
const int XBOX_B_BUTTON = 1;
}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<class CommandType>
PathFollowing<CommandType>::PathFollowing(Node::SharedPtr node)
: node_(std::move(node)), curvilinear_abscissa_goal_(-1), path_following_algo_(node_)
//  rviz_tool_("map","obstacle")
{
  declare_command_interface_configuration(node_, "cmd_output");

  declare_parameter_with_default(node_, "joy_start_button1", XBOX_X_BUTTON);
  declare_parameter_with_default(node_, "joy_start_button2", XBOX_Y_BUTTON);
  declare_parameter_with_default(node_, "joy_stop_button", XBOX_B_BUTTON);
  declare_parameter_with_default(node_, "joy_start_button1_desired_lateral_deviation", 0.);
  declare_parameter_with_default(node_, "joy_start_button2_desired_lateral_deviation", 0.);

  // TODO move all this params to PathFollowingAlgo
  declare_parameter_with_default(node_, "desired_lateral_deviation", 0.);
  declare_parameter_with_default(node_, "desired_linear_speed", 0.5);
  declare_parameter_with_default(node_, "desired_course_deviation", 0.);
  declare_parameter_with_default(node_, "use_path_velocity", false);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::init()
{
  auto interface_config = get_command_interface_configuration(node_, "cmd_output");
  cmd_interface_ = std::make_unique<VehicleInterface>(node_, std::move(interface_config));

  node_->get_parameter("joy_start_button1", joy_start_button1_id_);
  node_->get_parameter("joy_start_button2", joy_start_button2_id_);
  node_->get_parameter("joy_stop_button", joy_stop_button_id_);
  node_->get_parameter(
    "joy_start_button1_desired_lateral_deviation", joy_start_button1_desired_lateral_deviation_);
  node_->get_parameter(
    "joy_start_button2_desired_lateral_deviation", joy_start_button2_desired_lateral_deviation_);
  node_->get_parameter("desired_lateral_deviation", desired_lateral_deviation_);

  // this->initActionServer_(nh, private_nh);
  // this->initController_(nh, private_nh, robot_nh);
  init_algo_();

  using namespace std::placeholders;
  auto matching_cb = std::bind(&PathFollowing::process_matching_info_, this, _1);
  matching_sub_ = node_->create_subscription<romea_path_msgs::msg::PathMatchingInfo2D>(
    "path_matching/info", reliable(1), std::move(matching_cb));

  auto odom_cb = std::bind(&PathFollowing::process_odometry_, this, _1);
  odometry_sub_ =
    node_->create_subscription<OdometryMeasureMsg>("odometry", reliable(1), std::move(odom_cb));

  auto joystick_cb = std::bind(&PathFollowing::process_joystick_, this, _1);
  joystick_sub_ =
    node_->create_subscription<sensor_msgs::msg::Joy>("joy", reliable(1), std::move(joystick_cb));

  auto linear_speed_cb = std::bind(&PathFollowing::process_desired_linear_speed_, this, _1);
  desired_linear_speed_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "~/desired_linear_speed", reliable(1), std::move(linear_speed_cb));

  auto lateral_dev_cb = std::bind(&PathFollowing::process_desired_linear_speed_, this, _1);
  desired_lateral_deviation_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "~/desired_lateral_deviation", reliable(1), std::move(lateral_dev_cb));

  // fsm_service_ =
  //   private_nh.advertiseService("fsm_service", &PathFollowing<CommandType>::serviceCallback_, this);

  path_following_algo_.setDesiredLateralDeviation(desired_lateral_deviation_);
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowing<CommandType>::process_odometry_(const OdometryMeasureMsg & msg)
{
  path_following_algo_.setSteeringAngles(msg);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::process_desired_linear_speed_(
  std_msgs::msg::Float64::ConstSharedPtr msg)
{
  path_following_algo_.setDesiredLinearSpeed(msg->data);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::process_desired_lateral_deviation_(
  const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  path_following_algo_.setDesiredLateralDeviation(msg->data);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::init_algo_()
{
  auto linear_speed = get_parameter<double>(node_, "desired_linear_speed");
  auto lateral_dev = get_parameter<double>(node_, "desired_lateral_deviation");
  auto course_dev = get_parameter<double>(node_, "desired_course_deviation");
  auto use_path_vel = get_parameter<bool>(node_, "use_path_velocity");

  path_following_algo_.init();
  path_following_algo_.setDesiredLinearSpeed(linear_speed);
  path_following_algo_.setDesiredLateralDeviation(lateral_dev);
  path_following_algo_.setDesiredCourseDeviation(course_dev);
  path_following_algo_.usePathVelocity(use_path_vel);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::process_matching_info_(
  romea_path_msgs::msg::PathMatchingInfo2D::ConstSharedPtr msg)
{
  //  std::cout << "processMatchingInfo" << std::endl;
  //  std::cout << *msg<<std::endl;

  // const double path_length = msg.path_length;
  // const double curvilinear_abscissa =
  //   msg.matched_points[msg.tracked_matched_point_index].frenet_pose.curvilinear_abscissa;
  //
  // if (action_server_ != nullptr && action_server_->isActive()) {
  //   manageActionController(curvilinear_abscissa, path_length);
  // } else {
  //   if (curvilinear_abscissa > path_length - 0.5) {
  //     //      this->cmd_interface_.stop(true);
  //   }
  // }

  if (cmd_interface_->is_started()) {
    CommandType command = path_following_algo_.computeCommand(*msg);
    cmd_interface_->send_command(command);
  }
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::process_joystick_(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  if (msg->buttons[joy_start_button1_id_]) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("joystick"), "start button 1");
    path_following_algo_.setDesiredLateralDeviation(joy_start_button1_desired_lateral_deviation_);
    cmd_interface_->start();
  }

  if (msg->buttons[joy_start_button2_id_]) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("joystick"), "start button 2");
    path_following_algo_.setDesiredLateralDeviation(joy_start_button2_desired_lateral_deviation_);
    cmd_interface_->start();
  }

  if (msg->buttons[joy_stop_button_id_]) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("joystick"), "stop button");
    cmd_interface_->stop(true);
    // preemptActionServer("Stop by joystisk");
  }
}

template<class CommandType>
void PathFollowing<CommandType>::start()
{
  path_following_algo_.reset();
  cmd_interface_->start();
  RCLCPP_INFO_STREAM(node_->get_logger(), "command started");
  path_following_algo_.setDesiredLateralDeviation(desired_lateral_deviation_);
}

template<class CommandType>
void PathFollowing<CommandType>::stop()
{
  cmd_interface_->stop(true);
  RCLCPP_INFO_STREAM(node_->get_logger(), "command stopped");
}

// //-----------------------------------------------------------------------------
// template <class CommandType>
// void PathFollowing<CommandType>::goalCallback_()
// {
//   std::string goal = action_server_->acceptNewGoal()->goal;
//   if (goal.compare("end") == 0) {
//     curvilinear_abscissa_goal_ = -1;
//   } else {
//     curvilinear_abscissa_goal_ = std::stod(goal);
//   }
// }
//
// //-----------------------------------------------------------------------------
// template <class CommandType>
// void PathFollowing<CommandType>::preemptCallback_()
// {
//   action_server_->setPreempted();
//   this->cmd_interface_.stop(true);
// }
//
// //-----------------------------------------------------------------------------
// template <class CommandType>
// void PathFollowing<CommandType>::timeoutCallback_()
// {
//   path_following_algo_.reset();
//   abortActionServer("No path matching available");
//   //  cmd_pub_.stop(true);
// }
//
// //-----------------------------------------------------------------------------
// template <class CommandType>
// bool PathFollowing<CommandType>::serviceCallback_(
//   romea_fsm_msgs::FSMService::Request & request, romea_fsm_msgs::FSMService::Response & response)
// {
//   if (request.command.compare("start") == 0) {
//     path_following_algo_.reset();
//     this->cmd_interface_.start();
//     response.success = true;
//     ROS_INFO_STREAM("command started");
//     path_following_algo_.setDesiredLateralDeviation(desired_lateral_deviation_);
//   } else if (request.command.compare("stop") == 0) {
//     // stop and optionnally reset if the "reset" argument is specified
//     bool reset = request.command_arguments == "reset";
//
//     this->cmd_interface_.stop(reset);
//     response.success = true;
//     ROS_INFO_STREAM("command stopped (reset = " << reset << ')');
//   } else {
//     return false;
//   }
//
//   return true;
// }
//
// //-----------------------------------------------------------------------------
// template <class CommandType>
// void PathFollowing<CommandType>::manageActionController(
//   const double & curvilinear_abscissa, const double & path_length)
// {
//   double curvilinear_abscissa_goal =
//     (curvilinear_abscissa_goal_.load() < 0 ? path_length : curvilinear_abscissa_goal_.load()) - 0.5;
//   double distance_to_goal = curvilinear_abscissa_goal - curvilinear_abscissa;
//
//   romea_control_msgs::FollowPathFeedback action_feeback;
//   action_feeback.distance_to_goal = distance_to_goal;
//   romea_control_msgs::FollowPathResult action_result;
//   action_result.curvilinear_abscissa = curvilinear_abscissa;
//
//   //  if(curvilinear_abscissa>=curvilinear_abscissa_goal)
//   //  {
//   //    action_server_->setSucceeded(action_result);
//   //  }
//   //  else if(curvilinear_abscissa>=path_length)
//   //  {
//   //    action_server_->setAborted(action_result);
//   //  }
// }

template class PathFollowing<core::TwoAxleSteeringCommand>;
template class PathFollowing<core::OneAxleSteeringCommand>;

}  // namespace ros2
}  // namespace romea
