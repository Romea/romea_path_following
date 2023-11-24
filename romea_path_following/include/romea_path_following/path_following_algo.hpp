#ifndef __PathFollowingAlgo_HPP__
#define __PathFollowingAlgo_HPP__

//std
#include <atomic>
#include <memory>

// ros
#include <rclcpp_lifecycle/lifecycle_node.hpp>

//romea
//- #include <ros/VehicleControlTraits.hpp>
#include <romea_core_common/log/SimpleFileLogger.hpp>
#include <romea_core_control/command/FollowTrajectoryClassicSliding.hpp>
#include <romea_core_control/command/FollowTrajectoryPredictiveSliding.hpp>
#include <romea_core_control/observer/SlidingObserverCinematicLinearTangent.hpp>
#include <romea_core_control/observer/SlidingObserverCinematicLyapunov.hpp>
#include <romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp>
#include <romea_path_utils/path_matching_info_conversions.hpp>
#include <romea_mobile_base_utils/control/command_traits.hpp>

#ifdef ROMEA_CONTROL_PRIVATE
#include <command/PrivateFollowTrajectoryCinematicPredictiveSliding.hpp>
#include <command/PrivateFollowTrajectoryDynamicPredictiveSliding.hpp>
#include <observer/PrivateSlidingObserverDynamicBackStepping.hpp>
#include <observer/PrivateSlidingObserverDynamicLyapunov.hpp>
#endif

#include "path_following_mode.hpp"

namespace romea
{
namespace ros2
{

template<class CommandType>
class PathFollowingAlgo
{
public:
  using Node = rclcpp_lifecycle::LifecycleNode;
  using OdometryMeasureMsg = typename CommandTraits<CommandType>::MeasureMsg;

  enum class FSMStatus
  {
    BIRD_DECELERATE,
    BIRD_WHEEL_ROTATION,
    BIRD_ACCELERATE,
    CLASSIC
  };

  enum class Command
  {
    CLASSIC,
    PREDICTIVE,
#ifdef ROMEA_CONTROL_PRIVATE
    CINEMATIC_PREDICTIVE,
    DYNAMIC_PREDICTIVE,
#endif
    UNKNOWN
  };

  enum class Observer
  {
    CINEMATIC,
    CINEMATIC_LYAPUNOV,
#ifdef ROMEA_CONTROL_PRIVATE
    DYNAMIC,
    DYNAMIC_LYAPUNOV,
#endif
    WITHOUT
  };

  // clang-format off
  static constexpr auto OBSERVER_NS_CINEMATIC = "observers.cinematic_linear_tangent";
  static constexpr auto OBSERVER_NS_CINEMATIC_LYAPUNOV = "observers.cinematic_lyapunov";
  static constexpr auto OBSERVER_NS_DYNAMIC = "observers.dynamic_back_stepping";
  static constexpr auto OBSERVER_NS_DYNAMIC_LYAPUNOV = "observers.dynamic_lyapunov";
  static constexpr auto COMMAND_NS_CLASSIC = "commands.classic";
  static constexpr auto COMMAND_NS_PREDICTIVE = "commands.predictive";
  static constexpr auto COMMAND_NS_CINEMATIC_PREDICTIVE = "commands.cinematic_predictive";
  static constexpr auto COMMAND_NS_DYNAMIC_PREDICTIVE = "commands.dynamic_predictive";
  // clang-format on

public:
  PathFollowingAlgo(Node::SharedPtr node);

  void init();

  void reset();

  void setDesiredLinearSpeed(const double & linear_speed);

  void setDesiredLateralDeviation(const double & lateral_deviation);
  double getDesiredLateralDeviation() const {return desired_lateral_deviation_.load();}

  void setDesiredCourseDeviation(const double & course_deviation);

  void resetSetPoint();

  void setSteeringAngles(const OdometryMeasureMsg & msg);

  void usePathVelocity(bool enabled) {use_path_velocity_ = enabled;}

  CommandType computeCommand(const romea_path_msgs::msg::PathMatchingInfo2D & msg);

private:
  void initKinematicParameters_();
  void initFollowTrajectory_();
  void initObservers_();
  void initLogger_();

  void selectSlidingObserver_();
  void resetSlidingObservers_();
  void makeSlidingObserverCinematic_();
  void makeSlidingObserverCinematicLyapunov_();
  void getSlidindAngles_();
  void computeSlidingAngles_();
  void computeSlidingAnglesCinematic_();
  void computeSlidingAnglesCinematicLyapunov_();

  void selectCommand_();
  void makeFollowTrajectoryClassicSliding_();
  void makeFollowTrajectoryPredictiveSliding_();
  void computeSteeringAngles_();
  void computeSteeringAnglesClassicSliding_();
  void computeSteeringAnglesPredictiveSliding_();

#ifdef ROMEA_CONTROL_PRIVATE
  void makeSlidingObserverDynamic_();
  void makeSlidingObserverDynamicLyapunov_();
  void computeSlidingAnglesDynamic_();
  void computeSlidingAnglesDynamicLyapunov_();

  void makeFollowTrajectoryCinematicPredictiveSliding_();
  void makeFollowTrajectoryDynamicPredictiveSliding_();
  void computeSteeringAnglesCinematicPredictiveSliding_();
  void computeSteeringAnglesDynamicPredictiveSliding_();
#endif

  void computeLinearSpeedCommand_();
  void limitLinearSpeedCommand_();
  void computeKP_();

  void updateFSM_(const romea_path_msgs::msg::PathMatchingInfo2D & msg);
  void extractMatchedPointInfo(const romea_path_msgs::msg::PathMatchedPoint2D & msg);

  CommandType makeCommand_();

private:
  Node::SharedPtr node_;

  double default_desired_linear_speed_ = 0;
  std::atomic<double> desired_linear_speed_ = 0;
  std::atomic<double> desired_lateral_deviation_ = 0;
  std::atomic<double> desired_course_deviation_ = 0;
  std::atomic<double> curvilinear_abscissa_goal_ = -1;
  bool is_rear_steering_command_enabled_ = false;
  bool use_path_velocity_ = false;
  bool stop_at_the_end_ = true;

  core::MobileBaseInertia inertia_parameters_;
  double maximal_front_steering_angle_ = 0;
  double maximal_rear_steering_angle_ = 0;
  double wheelbase_ = 0;

  std::atomic<double> linear_speed_ = 0;
  std::atomic<double> angular_speed_ = 0;
  std::atomic<double> front_steering_angle_ = 0;
  std::atomic<double> rear_steering_angle_ = 0;

  romea::core::PathFrenetPose2D frenet_pose_;
  romea::core::PathPosture2D path_posture_;
  double future_curvature_ = 0;
  double path_length_ = 0;

  Observer selected_observer_ = Observer::WITHOUT;
  Observer desired_observer_;
  std::unique_ptr<core::SlidingObserverCinematicLinearTangent> sliding_observer_cinematic_;
  std::unique_ptr<core::SlidingObserverCinematicLyapunov> sliding_observer_cinematic_lyapunov_;
  core::FrontRearData sliding_angles_;

  Command selected_command_;
  Command desired_command_;
  std::unique_ptr<core::FollowTrajectoryClassicSliding> follow_trajectory_classic_sliding_;
  std::unique_ptr<core::FollowTrajectoryPredictiveSliding> follow_trajectory_predictive_sliding_;
  core::FrontRearData steering_angles_command_;

#ifdef ROMEA_CONTROL_PRIVATE
  std::unique_ptr<core::PrivateSlidingObserverDynamicBackStepping> sliding_observer_dynamic_;
  std::unique_ptr<core::PrivateSlidingObserverDynamicLyapunov> sliding_observer_dynamic_lyapunov_;
  std::unique_ptr<core::PrivateFollowTrajectoryCinematicPredictiveSliding>
  follow_trajectory_cinematic_predictive_sliding_;
  std::unique_ptr<core::PrivateFollowTrajectoryDynamicPredictiveSliding>
  follow_trajectory_dynamic_predictive_sliding_;
#endif

  double linear_speed_command_ = 0;
  double prev_linear_speed_command_ = 0;
  double minimal_linear_speed_command_ = 0;
  double linear_accel_limit_min_ = 0;
  double linear_accel_limit_max_ = 0;
  bool automatic_linear_speed_control_ = false;
  std::size_t section_index_last_{0};

  FSMStatus fsm_status = FSMStatus::CLASSIC;
  PathFollowingMode path_following_mode_;
  core::SimpleFileLogger debug_logger_;
};

}  // namespace ros2
}  // namespace romea

#endif
