//std
#include <cmath>
#include <functional>
#include <random>
#include <type_traits>

//romea
#include <romea_common_utils/params/algorithm_parameters.hpp>
#include <romea_core_common/math/Algorithm.hpp>
#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>
#include <romea_following_utils/command_factory.hpp>
#include <romea_following_utils/observer_factory.hpp>
#include <romea_mobile_base_utils/params/mobile_base_parameters.hpp>
#include <romea_path_utils/path_frenet_pose2d_conversions.hpp>
#include <romea_path_utils/path_posture2d_conversions.hpp>

#include "romea_path_following/path_following_algo.hpp"
#include "romea_path_following/path_following_mode.hpp"

#ifdef ROMEA_CONTROL_PRIVATE
#include <ros/PrivateCommandFactory.hpp>
#include <ros/PrivateObserverFactory.hpp>
#endif

namespace romea
{

//-----------------------------------------------------------------------------
template<class CommandType>
PathFollowingAlgo<CommandType>::PathFollowingAlgo(Node::SharedPtr node) : node_(std::move(node))
{
  if constexpr (std::is_same_v<CommandType, OneAxleSteeringCommand>) {
    declare_mobile_base_info_2FWS2RWD(node_, "base");
  } else if constexpr (std::is_same_v<CommandType, TwoAxleSteeringCommand>) {
    declare_mobile_base_info_4WS4WD(node_, "base");
  }

  declare_debug(node_);
  declare_sliding_observer_cinematic_linear_tangent_parameters(node_, OBSERVER_NS_CINEMATIC);
  declare_sliding_observer_cinematic_lyapunov_parameters(node_, OBSERVER_NS_CINEMATIC_LYAPUNOV);
#ifdef ROMEA_CONTROL_PRIVATE
  declare_sliding_observer_dynamic_back_stepping_parameters(node_, OBSERVER_NS_DYNAMIC);
  declare_sliding_observer_dynamic_lyapunov_parameters(node_, OBSERVER_NS_DYNAMIC_LYAPUNOV);
#endif
  declare_follow_trajectory_classic_sliding_parameters(node_, COMMAND_NS_CLASSIC);
  declare_follow_trajectory_predictive_sliding_parameters(node_, COMMAND_NS_PREDICTIVE);
#ifdef ROMEA_CONTROL_PRIVATE
  declare_follow_trajectory_cinematic_predictive_sliding_parameters(
    node_, COMMAND_NS_CINEMATIC_PREDICTIVE);
  declare_follow_trajectory_dynamic_predictive_sliding_parameters(
    node_, COMMAND_NS_DYNAMIC_PREDICTIVE);
#endif

  node_->declare_parameter("rear_steering_command_enabled", true);
  node_->declare_parameter("stop_at_the_end", true);
  node_->declare_parameter("maximal_steering_angle", M_PI_2);
  node_->declare_parameter("sliding_observer", rclcpp::PARAMETER_STRING);
  node_->declare_parameter("command", rclcpp::PARAMETER_STRING);
  node_->declare_parameter("minimal_linear_speed_command", rclcpp::PARAMETER_DOUBLE);
  node_->declare_parameter("automatic_linear_speed_control", rclcpp::PARAMETER_BOOL);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::init()
{
  initKinematicParameters_();
  initFollowTrajectory_();
  initObservers_();
  initLogger_();

  node_->get_parameter("stop_at_the_end", stop_at_the_end_);
}

//-----------------------------------------------------------------------------
template<>
void PathFollowingAlgo<TwoAxleSteeringCommand>::initKinematicParameters_()
{
  auto base_info = get_mobile_base_info_4WS4WD(node_, "base");
  wheelbase_ = base_info.geometry.axlesDistance;
  inertia_parameters_ = base_info.inertia;

  node_->get_parameter("rear_steering_command_enabled", is_rear_steering_command_enabled_);

  double max_steering_angle = get_parameter<double>(node_, "maximal_steering_angle");
  maximal_front_steering_angle_ =
    std::min(max_steering_angle, base_info.wheelsSteeringControl.command.maximalAngle);
  maximal_rear_steering_angle_ = maximal_front_steering_angle_;
}

//-----------------------------------------------------------------------------
template<>
void PathFollowingAlgo<OneAxleSteeringCommand>::initKinematicParameters_()
{
  auto base_info = get_mobile_base_info_2FWS2RWD(node_, "base");
  wheelbase_ = base_info.geometry.axlesDistance;
  inertia_parameters_ = base_info.inertia;

  is_rear_steering_command_enabled_ = false;

  double max_steering_angle = get_parameter<double>(node_, "maximal_steering_angle");
  maximal_front_steering_angle_ =
    std::min(max_steering_angle, base_info.frontWheelsSteeringControl.command.maximalAngle);
  maximal_rear_steering_angle_ = maximal_front_steering_angle_;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::initLogger_()
{
  if (get_debug(node_)) {
    debug_logger_.init(get_log_filename(node_));
  }
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::setDesiredLinearSpeed(const double & linear_speed)
{
  default_desired_linear_speed_ = linear_speed;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::setDesiredLateralDeviation(const double & lateral_deviation)
{
  desired_lateral_deviation_ = lateral_deviation;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::setDesiredCourseDeviation(const double & course_deviation)
{
  desired_course_deviation_ = course_deviation;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::resetSetPoint()
{
  desired_linear_speed_ = 0;
  desired_lateral_deviation_ = 0;
  desired_course_deviation_ = 0;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::reset()
{
  resetSetPoint();
  resetSlidingObservers_();
}

//-----------------------------------------------------------------------------
template<class CommandType>
CommandType PathFollowingAlgo<CommandType>::computeCommand(
  const romea_path_msgs::msg::PathMatchingInfo2D & msg)
{
  linear_speed_ = msg.twist.linear_speeds.x;
  angular_speed_ = msg.twist.angular_speed;
  path_length_ = msg.path_length;
  updateFSM_(msg);

  switch (fsm_status) {
    case FSMStatus::CLASSIC: {
      size_t id = msg.tracked_matched_point_index;
      extractMatchedPointInfo(msg.matched_points[id]);
      computeLinearSpeedCommand_();
    } break;
    case FSMStatus::BIRD_DECELERATE: {
      extractMatchedPointInfo(msg.matched_points[0]);
      linear_speed_command_ = 0;
    } break;
    case FSMStatus::BIRD_WHEEL_ROTATION: {
      extractMatchedPointInfo(msg.matched_points[0]);
      linear_speed_command_ = 0;
    } break;
    case FSMStatus::BIRD_ACCELERATE: {
      extractMatchedPointInfo(msg.matched_points[0]);
      computeLinearSpeedCommand_();
    } break;
    default:
      break;
  };

  debug_logger_.addEntry("stamp", rclcpp::Time{msg.header.stamp}.seconds());
  debug_logger_.addEntry("fsm_status", static_cast<int>(fsm_status));
  debug_logger_.addEntry("path_length", msg.path_length);
  debug_logger_.addEntry("curvilinear_abscissa", frenet_pose_.curvilinearAbscissa);
  debug_logger_.addEntry("lateral_deviation", frenet_pose_.lateralDeviation);
  debug_logger_.addEntry("course_deviation", frenet_pose_.courseDeviation);
  debug_logger_.addEntry("path_course", path_posture_.course);
  debug_logger_.addEntry("path_curvature", path_posture_.curvature);
  debug_logger_.addEntry("path_futur_curvature", future_curvature_);
  debug_logger_.addEntry("rear_steering_angle", rear_steering_angle_);
  debug_logger_.addEntry("front_steering_angle", front_steering_angle_);
  debug_logger_.addEntry("linear_speed", linear_speed_);
  debug_logger_.addEntry("angular_speed", angular_speed_);
  debug_logger_.addEntry("linear_speed_command", linear_speed_command_);
  debug_logger_.addEntry("desired_lateral_deviation", desired_lateral_deviation_);

  computeKP_();
  computeSlidingAngles_();
  computeSteeringAngles_();
  return makeCommand_();
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::selectSlidingObserver_()
{
  std::string observer_name = get_parameter<std::string>(node_, "sliding_observer");

  if (observer_name == "cinematic_linear_tangent") {
    desired_observer_ = Observer::CINEMATIC;
  } else if (observer_name == "cinematic_lyapunov") {
    desired_observer_ = Observer::CINEMATIC_LYAPUNOV;
  }
#ifdef ROMEA_CONTROL_PRIVATE
  else if (observer_name == "dynamic") {
    desired_observer_ = Observer::DYNAMIC;
  } else if (observer_name == "dynamic_lyapunov") {
    desired_observer_ = Observer::DYNAMIC_LYAPUNOV;
  }
#endif
  else if (observer_name == "without") {
    desired_observer_ = Observer::WITHOUT;
  } else {
    throw std::runtime_error("No available observer is selected");
  }
  // ROS_ERROR_STREAM("observer_name " << observer_name);
  selected_observer_ = desired_observer_;
  //  ROS_ERROR_STREAM("selected_observer_ " << selected_observer_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::initObservers_()
{
  selectSlidingObserver_();
  makeSlidingObserverCinematic_();
  makeSlidingObserverCinematicLyapunov_();
#ifdef ROMEA_CONTROL_PRIVATE
  makeSlidingObserverDynamic_();
  makeSlidingObserverDynamicLyapunov_();
#endif
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::makeSlidingObserverCinematic_()
{
  sliding_observer_cinematic_ = make_observer<SlidingObserverCinematicLinearTangent>(
    node_, OBSERVER_NS_CINEMATIC, 0.1, wheelbase_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::makeSlidingObserverCinematicLyapunov_()
{
  sliding_observer_cinematic_lyapunov_ = make_observer<SlidingObserverCinematicLyapunov>(
    node_, OBSERVER_NS_CINEMATIC_LYAPUNOV, 0.1, wheelbase_);
}

#ifdef ROMEA_CONTROL_PRIVATE
//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::makeSlidingObserverDynamic_()
{
  sliding_observer_dynamic_ = make_observer<PrivateSlidingObserverDynamicBackStepping>(
    node_, OBSERVER_NS_DYNAMIC, 0.1, wheelbase_, inertia_parameters_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::makeSlidingObserverDynamicLyapunov_()
{
  sliding_observer_dynamic_lyapunov_ = make_observer<PrivateSlidingObserverDynamicLyapunov>(
    node_, OBSERVER_NS_DYNAMIC_LYAPUNOV, 0.1, wheelbase_, inertia_parameters_);
}
#endif

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::resetSlidingObservers_()
{
  sliding_observer_cinematic_->reset();
  sliding_observer_cinematic_lyapunov_->reset();
#ifdef ROMEA_CONTROL_PRIVATE
  sliding_observer_dynamic_->reset();
  sliding_observer_dynamic_lyapunov_->reset();
#endif
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::makeFollowTrajectoryClassicSliding_()
{
  follow_trajectory_classic_sliding_ =
    make_command<FollowTrajectoryClassicSliding>(node_, COMMAND_NS_CLASSIC, wheelbase_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::makeFollowTrajectoryPredictiveSliding_()
{
  follow_trajectory_predictive_sliding_ =
    make_command<FollowTrajectoryPredictiveSliding>(node_, COMMAND_NS_PREDICTIVE, wheelbase_);
}

#ifdef ROMEA_CONTROL_PRIVATE
//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::makeFollowTrajectoryCinematicPredictiveSliding_(
  ros::NodeHandle & private_nh)
{
  if (!is_rear_steering_command_enabled_) {
    throw std::runtime_error(
      "Rear steering command must be enabled when using cinematic predictive command");
  }

  using Command = PrivateFollowTrajectoryCinematicPredictiveSliding;
  follow_trajectory_cinematic_predictive_sliding_ =
    make_command<Command>(node_, COMMAND_NS_CINEMATIC_PREDICTIVE, wheelbase_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::makeFollowTrajectoryDynamicPredictiveSliding_(
  ros::NodeHandle & private_nh)
{
  if (is_rear_steering_command_enabled_) {
    throw std::runtime_error(
      "Rear steering command must be disabled when using dynamic predictive command");
  }

  using Command = PrivateFollowTrajectoryDynamicPredictiveSliding;
  follow_trajectory_dynamic_predictive_sliding_ =
    make_command<Command>(node_, COMMAND_NS_DYNAMIC_PREDICTIVE, wheelbase_);
}
#endif

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::selectCommand_()
{
  std::string command_name = get_parameter<std::string>(node_, "command");

  if (command_name == "classic") {
    desired_command_ = Command::CLASSIC;
  } else if (command_name == "predictive") {
    desired_command_ = Command::PREDICTIVE;
  }
#ifdef ROMEA_CONTROL_PRIVATE
  else if (command_name == "cinematic_predictive") {
    desired_command_ = Command::CINEMATIC_PREDICTIVE;
  } else if (command_name.find("dynamic_predictive") != std::string::npos) {
    if (desired_observer_ != Observer::DYNAMIC_LYAPUNOV) {
      throw std::runtime_error(
        "Dynamic Lyapunov sliding observer must be selected when using dynamic predictive "
        "command");
    }

    if (command_name.find("one_steering") != std::string::npos) {
      desired_command_ = Command::DYNAMIC_PREDICTIVE;
    } else {
      throw std::runtime_error("No available command is selected");
    }
  }
#endif
  else {
    throw std::runtime_error("No available command is selected");
  }

  selected_command_ = desired_command_;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::initFollowTrajectory_()
{
  selectCommand_();
  makeFollowTrajectoryClassicSliding_();
  makeFollowTrajectoryPredictiveSliding_();

  //   switch (selected_command_)
  //   {
  //   case Command::CLASSIC:
  //     makeFollowTrajectoryClassicSliding_(private_nh);
  //     break;
  //   case Command::PREDICTIVE:
  //     makeFollowTrajectoryPredictiveSliding_(private_nh);
  //     break;
  // #ifdef ROMEA_CONTROL_PRIVATE
  //   case Command::CINEMATIC_PREDICTIVE:
  //     makeFollowTrajectoryCinematicPredictiveSliding_(private_nh);
  //     break;
  //   case Command::DYNAMIC_PREDICTIVE:
  //     makeFollowTrajectoryDynamicPredictiveSliding_(private_nh);
  //     break;
  // #endif
  //   default:
  //     throw std::runtime_error("No available command is selected");
  //     break;
  //   }

  minimal_linear_speed_command_ = get_parameter<double>(node_, "minimal_linear_speed_command");
  automatic_linear_speed_control_ = get_parameter<bool>(node_, "automatic_linear_speed_control");
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSlidingAnglesCinematic_()
{
  sliding_observer_cinematic_->update(
    frenet_pose_.lateralDeviation,
    sign(linear_speed_) * frenet_pose_.courseDeviation,
    path_posture_.curvature,
    linear_speed_,
    front_steering_angle_,
    rear_steering_angle_);
  if (frenet_pose_.curvilinearAbscissa < 5)
    sliding_observer_cinematic_->initObserver_(
      frenet_pose_.lateralDeviation, frenet_pose_.courseDeviation);

  //  if (frenet_pose_.curvilinearAbscissa<5)
  //      sliding_observer_cinematic_->initObserver_(frenet_pose_.lateralDeviation,frenet_pose_.courseDeviation);

  debug_logger_.addEntry("Elat4", sliding_observer_cinematic_->getLateralDeviation());
  debug_logger_.addEntry("Ecap4", sliding_observer_cinematic_->getCourseDeviation());
  debug_logger_.addEntry("BR", sliding_observer_cinematic_->getRearSlidingAngle());
  debug_logger_.addEntry("BF", sliding_observer_cinematic_->getFrontSlidingAngle());
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSlidingAnglesCinematicLyapunov_()
{
  double x =
    path_posture_.position.x() - std::sin(path_posture_.course) * frenet_pose_.lateralDeviation;
  double y =
    path_posture_.position.y() + std::cos(path_posture_.course) * frenet_pose_.lateralDeviation;
  double course = path_posture_.course + sign(linear_speed_) * frenet_pose_.courseDeviation;

  debug_logger_.addEntry("x", x);
  debug_logger_.addEntry("y", y);
  debug_logger_.addEntry("course", course);

  sliding_observer_cinematic_lyapunov_->update(
    x, y, course, linear_speed_, front_steering_angle_, rear_steering_angle_);
  if (frenet_pose_.curvilinearAbscissa < 5)
    sliding_observer_cinematic_lyapunov_->initObserverHandbooks_(x, y, course);

  //  if (frenet_pose_.curvilinearAbscissa<5)
  //      sliding_observer_cinematic_lyapunov_->initObserverHandbooks_(x,y,course);

  debug_logger_.addEntry("XObs", sliding_observer_cinematic_lyapunov_->getX());
  debug_logger_.addEntry("YObs", sliding_observer_cinematic_lyapunov_->getY());
  debug_logger_.addEntry("ThetaObs", sliding_observer_cinematic_lyapunov_->getTheta());
  debug_logger_.addEntry("BetaRHand", sliding_observer_cinematic_lyapunov_->getRearSlidingAngle());
  debug_logger_.addEntry("BetaFHand", sliding_observer_cinematic_lyapunov_->getFrontSlidingAngle());
}

#ifdef ROMEA_CONTROL_PRIVATE
//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSlidingAnglesDynamic_()
{
  sliding_observer_dynamic_->update(
    linear_speed_,
    angular_speed_,
    front_steering_angle_,
    rear_steering_angle_,
    0,
    sliding_observer_cinematic_->getFrontSlidingAngle(),
    sliding_observer_cinematic_->getRearSlidingAngle());

  debugLogger_.addEntry("HatVitLacet", sliding_observer_dynamic_->getAngularSpeed());
  debugLogger_.addEntry("HatBeta", sliding_observer_dynamic_->getGlobalSliding());
  debugLogger_.addEntry("HatVitLacet3", sliding_observer_dynamic_->getAngularSpeed3());
  debugLogger_.addEntry("HatBeta3", sliding_observer_dynamic_->getGlobalSliding3());
  debugLogger_.addEntry("BetaObj", sliding_observer_dynamic_->getGlobalSlidingOjective());
  debugLogger_.addEntry("BRDyn", sliding_observer_dynamic_->getRearSlidingAngle());
  debugLogger_.addEntry("BFDyn", sliding_observer_dynamic_->getFrontSlidingAngle());
  debugLogger_.addEntry("CF", sliding_observer_dynamic_->getRearCorneredStiffness());
  debugLogger_.addEntry("CR", sliding_observer_dynamic_->getFrontCorneredStiffness());
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSlidingAnglesDynamicLyapunov_()
{
  sliding_observer_dynamic_lyapunov_->update(
    frenet_pose_.lateralDeviation,
    frenet_pose_.courseDeviation,
    path_posture_.curvature,
    linear_speed_,
    angular_speed_,
    front_steering_angle_,
    rear_steering_angle_,
    0);

  debugLogger_.addEntry("EcLatG", sliding_observer_dynamic_lyapunov_->getLateralDeviation());
  debugLogger_.addEntry("EcAngG", sliding_observer_dynamic_lyapunov_->getCourseDeviation());
  debugLogger_.addEntry("CapDerivG", sliding_observer_dynamic_lyapunov_->getAngularSpeed());
  debugLogger_.addEntry("BetaFG", sliding_observer_dynamic_lyapunov_->getRearSlidingAngle());
  debugLogger_.addEntry("BetaRG", sliding_observer_dynamic_lyapunov_->getFrontSlidingAngle());
  debugLogger_.addEntry("CFG", sliding_observer_dynamic_lyapunov_->getRearCorneredStiffness());
  debugLogger_.addEntry("CRG", sliding_observer_dynamic_lyapunov_->getFrontCorneredStiffness());
}
#endif

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::getSlidindAngles_()
{
  // ROS_ERROR_STREAM("Glissement ");

  switch (selected_observer_) {
    case Observer::CINEMATIC:
      // ROS_ERROR_STREAM("Glissement cinematique");
      sliding_angles_ = sliding_observer_cinematic_->getSlidingAngles();
      break;
    case Observer::CINEMATIC_LYAPUNOV:
      // ROS_ERROR_STREAM("Glissement cinematique Lyapunov");
      sliding_angles_ = sliding_observer_cinematic_lyapunov_->getSlidingAngles();
      break;
#ifdef ROMEA_CONTROL_PRIVATE
    case Observer::DYNAMIC:
      sliding_angles_ = sliding_observer_dynamic_->getSlidingAngles();
      break;
    case Observer::DYNAMIC_LYAPUNOV:
      sliding_angles_ = sliding_observer_dynamic_lyapunov_->getSlidingAngles();
      break;
#endif
    default:
      // ROS_ERROR_STREAM("DEFAUT");
      break;
  }
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSlidingAngles_()
{
  if (path_following_mode_ == PathFollowingMode::ON_APPROACH) {
    resetSlidingObservers_();
  }

  computeSlidingAnglesCinematic_();
  computeSlidingAnglesCinematicLyapunov_();
#ifdef ROMEA_CONTROL_PRIVATE
  computeSlidingAnglesDynamic_();
  computeSlidingAnglesDynamicLyapunov_();
#endif

  getSlidindAngles_();
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSteeringAnglesClassicSliding_()
{
  steering_angles_command_ = follow_trajectory_classic_sliding_->computeSteeringAngles(
    frenet_pose_.lateralDeviation,
    frenet_pose_.courseDeviation,
    path_posture_.curvature,
    sliding_angles_.front,
    sliding_angles_.rear,
    rear_steering_angle_,
    maximal_front_steering_angle_,
    maximal_rear_steering_angle_,
    desired_lateral_deviation_,
    desired_course_deviation_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSteeringAnglesPredictiveSliding_()
{
  steering_angles_command_ = follow_trajectory_predictive_sliding_->computeSteeringAngles(
    frenet_pose_.lateralDeviation,
    frenet_pose_.courseDeviation,
    path_posture_.curvature,
    future_curvature_,
    front_steering_angle_,
    rear_steering_angle_,
    sliding_angles_.front,
    sliding_angles_.rear,
    maximal_front_steering_angle_,
    maximal_rear_steering_angle_,
    desired_lateral_deviation_,
    desired_course_deviation_,
    desired_lateral_deviation_);
}

#ifdef ROMEA_CONTROL_PRIVATE
//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSteeringAnglesCinematicPredictiveSliding_()
{
  steering_angles_command_ = follow_trajectory_cinematic_predictive_sliding_->computeSteeringAngles(
    frenet_pose_.lateralDeviation,
    frenet_pose_.courseDeviation,
    path_posture_.curvature,
    future_curvature_,
    linear_speed_,
    front_steering_angle_,
    rear_steering_angle_,
    sliding_angles_.front,
    sliding_angles_.rear,
    maximal_steeering_angle_,
    desired_lateral_deviation_,
    desired_course_deviation_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSteeringAnglesDynamicPredictiveSliding_()
{
  steering_angles_command_.front =
    follow_trajectory_dynamic_predictive_sliding_->computeFrontSteeringAngle(
      frenet_pose_.lateralDeviation,
      frenet_pose_.courseDeviation,
      path_posture_.curvature,
      future_curvature_,
      linear_speed_,
      angular_speed_,
      front_steering_angle_,
      rear_steering_angle_,
      sliding_angles_.front,
      sliding_angles_.rear,
      sliding_observer_dynamic_lyapunov_->getFrontCorneredStiffness(),
      sliding_observer_dynamic_lyapunov_->getRearCorneredStiffness(),
      maximal_steeering_angle_,
      desired_lateral_deviation_);
}
#endif

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeSteeringAngles_()
{
  //std::cout << " selected_command_ " << int(selected_command_)<<std::endl;
  //std::cout << " selected_observer_ " << int(selected_observer_) << std::endl;

  switch (selected_command_) {
    case Command::CLASSIC:
      // ROS_ERROR_STREAM("Commande CLASSIC");
      computeSteeringAnglesClassicSliding_();
      break;
    case Command::PREDICTIVE:
      // ROS_ERROR_STREAM("Commande PREDICTIVE");
      computeSteeringAnglesPredictiveSliding_();
      break;
#ifdef ROMEA_CONTROL_PRIVATE
    case Command::CINEMATIC_PREDICTIVE:
      computeSteeringAnglesCinematicPredictiveSliding_();
      break;
    case Command::DYNAMIC_PREDICTIVE:
      computeSteeringAnglesDynamicPredictiveSliding_();
      break;
#endif
    default:
      break;
  }
}

//-----------------------------------------------------------------------------
template<>
void PathFollowingAlgo<OneAxleSteeringCommand>::setSteeringAngles(const OdometryMeasureMsg & msg)
{
  front_steering_angle_ = msg.measure.steering_angle;
  rear_steering_angle_ = 0;
}

//-----------------------------------------------------------------------------
template<>
void PathFollowingAlgo<TwoAxleSteeringCommand>::setSteeringAngles(const OdometryMeasureMsg & msg)
{
  front_steering_angle_ = msg.measure.front_steering_angle;
  rear_steering_angle_ = msg.measure.rear_steering_angle;
}

//-----------------------------------------------------------------------------
template<>
OneAxleSteeringCommand PathFollowingAlgo<OneAxleSteeringCommand>::makeCommand_()
{
  OneAxleSteeringCommand command;
  command.longitudinalSpeed = linear_speed_command_;
  command.steeringAngle = steering_angles_command_.front;
  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution(0, 0.05);
  command.steeringAngle += distribution(generator);

  debug_logger_.addEntry("rear_steering_command", 0);
  debug_logger_.addEntry("front_steering_command", command.steeringAngle);
  debug_logger_.addEntry("desired_linear_speed", desired_linear_speed_);
  debug_logger_.writeRow();

  return command;
}

//-----------------------------------------------------------------------------
template<>
TwoAxleSteeringCommand PathFollowingAlgo<TwoAxleSteeringCommand>::makeCommand_()
{
  TwoAxleSteeringCommand command;
  command.longitudinalSpeed = linear_speed_command_;
  command.frontSteeringAngle = steering_angles_command_.front;
  command.rearSteeringAngle = steering_angles_command_.rear * is_rear_steering_command_enabled_;
  debug_logger_.addEntry("rear_steering_command", command.rearSteeringAngle);
  debug_logger_.addEntry("front_steering_command", command.frontSteeringAngle);
  debug_logger_.addEntry("desired_linear_speed", desired_linear_speed_);
  debug_logger_.writeRow();
  return command;
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeLinearSpeedCommand_()
{
  linear_speed_command_ = desired_linear_speed_;
  if (path_following_mode_ == PathFollowingMode::CLASSIC_PATH_FOLLOWING) {
    double abscissa = frenet_pose_.curvilinearAbscissa;
    if (stop_at_the_end_ && (abscissa < 0 || abscissa > path_length_)) {
      linear_speed_command_ = 0;
    } else if (automatic_linear_speed_control_) {
      limitLinearSpeedCommand_();
    }
  }

  //  debug_logger_.addEntry("linear_speed_command",linear_speed_command_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::limitLinearSpeedCommand_()
{
  const double & EcartLat = frenet_pose_.lateralDeviation;
  const double & EcartAng = frenet_pose_.courseDeviation;
  const double & steering = front_steering_angle_;

  double flag = 1;
  if (fabs(steering) < 10 / 180. * M_PI) flag = 0;

  linear_speed_command_ =
    fabs(desired_linear_speed_) -
    2 * (EcartLat - desired_lateral_deviation_) * (EcartLat - desired_lateral_deviation_) -
    10 * EcartAng * EcartAng - 1 * flag * fabs(steering);
  if (linear_speed_command_ < minimal_linear_speed_command_) {
    linear_speed_command_ = minimal_linear_speed_command_;
  }

  linear_speed_command_ = std::copysign(linear_speed_command_, desired_linear_speed_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::computeKP_()
{
  double kp_max = 1.5;
  double kp_min = 0.2;
  double kp = kp_max;
  double speed = 0.4;
  // **** Ancienne version *****
  //  if(std::abs(desired_linear_speed_)>0.1)
  //  {
  //    double s=std::abs(linear_speed_)-speed+speed*linear_speed_/1.;
  //    if (s<0)
  //      s=0;

  //    kp = kp_max-(s/std::abs(1.))*(kp_max-kp_min);

  //    if (kp<kp_min)
  //      kp=kp_min;
  //    if (kp>kp_max)
  //      kp=kp_max;
  //  }
  // **************

  // Nouvelle version a tester
  if (std::abs(linear_speed_) > 0.4) {
    speed = std::abs(linear_speed_);
  }
  double RapportVitRot = 8;
  double T_omega = 1.5;
  kp = 8 / (RapportVitRot * T_omega * speed);
  if (kp < kp_min) kp = kp_min;
  if (kp > kp_max) kp = kp_max;
  // *******

  debug_logger_.addEntry("kpcomputed", kp);

  if (selected_command_ == Command::CLASSIC) {
    follow_trajectory_classic_sliding_->setFrontKP(kp);
  } else if (selected_command_ == Command::PREDICTIVE) {
    follow_trajectory_predictive_sliding_->setFrontKP(kp);
  }
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::extractMatchedPointInfo(
  const romea_path_msgs::msg::PathMatchedPoint2D & msg)
{
  if (std::isfinite(msg.desired_speed)) {
    if (use_path_velocity_) {
      desired_linear_speed_ = msg.desired_speed;
    } else {
      desired_linear_speed_ = std::copysign(1, msg.desired_speed) * default_desired_linear_speed_;
    }
  } else {
    desired_linear_speed_ = default_desired_linear_speed_;
  }

  if (desired_linear_speed_ >= 0) {
    selected_command_ = desired_command_;
    selected_observer_ = desired_observer_;

    path_posture_ = to_romea(msg.posture);
    frenet_pose_ = to_romea(msg.frenet_pose);
    future_curvature_ = msg.future_curvature;
  } else {
    // Moving backward only works with classic + without
    //    selected_command_ = Command::CLASSIC;
    //    selected_observer_ = Observer::WITHOUT;
    // Test
    selected_command_ = desired_command_;
    selected_observer_ = desired_observer_;

    // path_posture_=toRomea(msg.posture);
    // frenet_pose_=toRomea(msg.frenet_pose);
    // future_curvature_=msg.future_curvature;
    // frenet_pose_.courseDeviation *= -1;

    path_posture_ = reverse(to_romea(msg.posture));
    frenet_pose_ = reverse(to_romea(msg.frenet_pose));
    future_curvature_ = -msg.future_curvature;
  }

  section_index_last_ = msg.section_index;

  path_following_mode_ = deducePathFollowingMode(msg);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowingAlgo<CommandType>::updateFSM_(
  const romea_path_msgs::msg::PathMatchingInfo2D & msg)
{
  // std::cout << " start fsm update " << int(fsm_status) <<
  //   " isDesiredSpeedSignChange " << isDesiredSpeedSignChange(toRomea(msg.matched_points[0]),toRomea(msg.matched_points[1])) <<
  //   " section_index_last_ " << section_index_last_ << std::endl;

  switch (fsm_status) {
    case FSMStatus::CLASSIC:
      // if it's the last point
      if (section_index_last_ != msg.matched_points[0].section_index) {
        fsm_status = FSMStatus::BIRD_DECELERATE;
      }
      break;
    case FSMStatus::BIRD_DECELERATE:
      if (std::abs(linear_speed_) < 0.01) {
        fsm_status = FSMStatus::BIRD_WHEEL_ROTATION;
      }
      break;
    case FSMStatus::BIRD_WHEEL_ROTATION:
      if (
        std::abs(front_steering_angle_ - steering_angles_command_.front) < 0.05 &&
        (is_rear_steering_command_enabled_ *
           std::abs(rear_steering_angle_ - steering_angles_command_.rear) <
         0.05)) {
        fsm_status = FSMStatus::BIRD_ACCELERATE;
      }
      break;
    case FSMStatus::BIRD_ACCELERATE:
      if (msg.matched_points.size() == 1) {
        fsm_status = FSMStatus::CLASSIC;
      }
      break;
    default:
      break;
  }

  // std::cout << " end fsm update " << int(fsm_status)<< std::endl;
}

template class PathFollowingAlgo<TwoAxleSteeringCommand>;
template class PathFollowingAlgo<OneAxleSteeringCommand>;

}  // namespace romea
