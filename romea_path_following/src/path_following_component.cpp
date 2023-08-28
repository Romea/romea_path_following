#include "romea_path_following/path_following_component.hpp"

#include <romea_common_utils/params/node_parameters.hpp>

namespace romea
{

PathFollowingComponent::PathFollowingComponent(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("path_following", options)
{
  rcl_interfaces::msg::ParameterDescriptor frame_descr;
  frame_descr.description = "Frame type of the robot [4WS4WD, 2FWS2RWD]";
  declare_parameter("robot.odometry.frame", rclcpp::PARAMETER_STRING, std::move(frame_descr));
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_configure(
  const rclcpp_lifecycle::State &)
try {
  auto frame = romea::get_parameter<std::string>(shared_from_this(), "robot.odometry.frame");

  if (frame == "4WS4WD") {
    control_ = std::make_unique<PathFollowing<TwoAxleSteeringCommand>>(shared_from_this());
  } else if (frame == "2FWS2RWD") {
    control_ = std::make_unique<PathFollowing<OneAxleSteeringCommand>>(shared_from_this());
  } else {
    throw std::runtime_error(frame + " frame type  is not supported");
  }
  control_->init();

  return CallbackReturn::SUCCESS;

} catch (const std::runtime_error & e) {
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("config"), e.what());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_activate(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::FAILURE;
}

}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::PathFollowingComponent)
