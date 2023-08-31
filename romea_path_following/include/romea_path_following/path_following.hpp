#ifndef __PathFollowing_HPP__
#define __PathFollowing_HPP__

//std
#include <atomic>
#include <functional>
#include <memory>
#include <queue>

//ros
// #include <four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
// #include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/msg/joint_state.hpp>

//tbb
// #include "tbb/concurrent_priority_queue.h"

//romea
// #include <romea_control_msgs/FollowPathAction.h>
// #include <romea_fsm_msgs/FSMService.h>

#include <romea_mobile_base_utils/control/command_interface.hpp>
#include <romea_mobile_base_utils/control/command_traits.hpp>
#include <romea_path_utils/path_matching_info_conversions.hpp>
// #include <ros/VehicleControlActionServer.hpp>

#include "path_following_algo.hpp"
#include "path_following_mode.hpp"

namespace romea
{

class PathFollowingBase
{
public:
  virtual ~PathFollowingBase() = default;
  virtual void init() = 0;
  virtual void start() {}
  virtual void stop() {}
};


template<class CommandType>
class PathFollowing : public PathFollowingBase
{
public:
  using Node = rclcpp_lifecycle::LifecycleNode;
  using VehicleInterface = CommandInterface<CommandType>;
  using OdometryMeasureMsg = typename CommandTraits<CommandType>::MeasureMsg;

public:
  PathFollowing(Node::SharedPtr node);

  virtual ~PathFollowing() = default;

  void init() override;

protected:
  void init_algo_();

  void process_matching_info_(romea_path_msgs::msg::PathMatchingInfo2D::ConstSharedPtr msg);

  void process_desired_linear_speed_(std_msgs::msg::Float64::ConstSharedPtr msg);
  void process_desired_lateral_deviation_(std_msgs::msg::Float64::ConstSharedPtr msg);

  void process_odometry_(const OdometryMeasureMsg & msg);

  void process_joystick_(sensor_msgs::msg::Joy::ConstSharedPtr msg);

  void start() override;

  void stop() override;

  // void goalCallback_();
  //
  // void preemptCallback_();
  //
  // void timeoutCallback_();
  //
  // virtual bool serviceCallback_(
  //   romea_fsm_msgs::FSMService::Request & request,
  //   romea_fsm_msgs::FSMService::Response & response) override;
  //
  // virtual void manageActionController(
  //   const double & curvilinear_abscissa, const double & path_length);

protected:
  Node::SharedPtr node_;

  std::unique_ptr<VehicleInterface> cmd_interface_;
  rclcpp::SubscriptionBase::SharedPtr matching_sub_;
  rclcpp::SubscriptionBase::SharedPtr odometry_sub_;
  rclcpp::SubscriptionBase::SharedPtr desired_linear_speed_sub_;
  rclcpp::SubscriptionBase::SharedPtr desired_lateral_deviation_sub_;
  rclcpp::SubscriptionBase::SharedPtr joystick_sub_;
  // ros::ServiceServer fsm_service_;

  std::atomic<double> curvilinear_abscissa_goal_;
  PathFollowingAlgo<CommandType> path_following_algo_;

  double joy_start_button1_desired_lateral_deviation_;
  double joy_start_button2_desired_lateral_deviation_;
  double desired_lateral_deviation_;
  int joy_start_button1_id_;
  int joy_start_button2_id_;
  int joy_stop_button_id_;
};

}  // namespace romea

#endif
