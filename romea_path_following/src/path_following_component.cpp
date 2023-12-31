#include "romea_path_following/path_following_component.hpp"

#include <romea_common_utils/params/node_parameters.hpp>

namespace romea
{
namespace ros2
{

PathFollowingComponent::PathFollowingComponent(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>("path_following", options))
{
  using std::placeholders::_1;
  node_->register_on_configure(std::bind(&PathFollowingComponent::on_configure, this, _1));
  node_->register_on_activate(std::bind(&PathFollowingComponent::on_activate, this, _1));
  node_->register_on_deactivate(std::bind(&PathFollowingComponent::on_deactivate, this, _1));

  rcl_interfaces::msg::ParameterDescriptor frame_descr;
  frame_descr.description = "Frame type of the robot [4WS4WD, 2FWS2RWD]";
  node_->declare_parameter("base.type", rclcpp::PARAMETER_STRING, std::move(frame_descr));

  rcl_interfaces::msg::ParameterDescriptor autoconf_descr;
  autoconf_descr.description = "Automatic configuration when the node is created";
  node_->declare_parameter("autoconfigure", false, std::move(autoconf_descr));

  rcl_interfaces::msg::ParameterDescriptor autostart_descr;
  autostart_descr.description = "Automatically start the robot when the node is configured";
  node_->declare_parameter("autostart", false, std::move(autostart_descr));

  auto frame = get_parameter<std::string>(node_, "base.type");
  if (frame == "4WS4WD") {
    control_ = std::make_unique<PathFollowing<core::TwoAxleSteeringCommand>>(node_);
  } else if (frame == "2FWS2RWD") {
    control_ = std::make_unique<PathFollowing<core::OneAxleSteeringCommand>>(node_);
  } else if (frame == "2FWS4WD") {
    control_ = std::make_unique<PathFollowing<core::OneAxleSteeringCommand>>(node_);
  } else {
    throw std::runtime_error(frame + " frame type  is not supported");
  }

  if (get_parameter<bool>(node_, "autoconfigure")) {
    auto state = node_->configure();
    if (get_parameter<bool>(node_, "autostart") && state.label() == "inactive") {
      node_->activate();
    }
  }
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_configure(
  const rclcpp_lifecycle::State &)
try
{
  control_->init();
  RCLCPP_INFO(node_->get_logger(), "configured");
  return CallbackReturn::SUCCESS;

} catch (const std::runtime_error & e) {
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("config"), e.what());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_activate(
  const rclcpp_lifecycle::State &)
{
  control_->start();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  control_->stop();
  return CallbackReturn::SUCCESS;
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::PathFollowingComponent)
