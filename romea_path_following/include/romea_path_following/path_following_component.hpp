#ifndef __PathFollowingComponent_HPP__
#define __PathFollowingComponent_HPP__

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "romea_path_following/path_following.hpp"

namespace romea
{
namespace ros2
{

class PathFollowingComponent
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  PathFollowingComponent(const rclcpp::NodeOptions & options);

  auto get_node_base_interface() const {return node_->get_node_base_interface();}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<PathFollowingBase> control_;
  bool autostart_;
};

}  // namespace ros2
}  // namespace romea

#endif
