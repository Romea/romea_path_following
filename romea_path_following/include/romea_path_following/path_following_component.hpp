#ifndef __PathFollowingComponent_HPP__
#define __PathFollowingComponent_HPP__

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "romea_path_following/path_following.hpp"

namespace romea
{

class PathFollowingComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  PathFollowingComponent(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

private:
  std::unique_ptr<PathFollowingBase> control_;
};

}  // namespace romea

#endif
