#ifndef romea_PathMatchingMode_hpp
#define romea_PathMatchingMode_hpp

//romea
#include <romea_path_msgs/msg/path_matched_point2_d.hpp>

//std
#include <string>

namespace romea
{

enum class PathFollowingMode
{
  CLASSIC_PATH_FOLLOWING,
  PLATOON_PATH_FOLLOWING,
  ON_APPROACH
};

PathFollowingMode deducePathFollowingMode(
  const romea_path_msgs::msg::PathMatchedPoint2D & matched_point_msg);

std::string toString(const PathFollowingMode & status);

}  //End of namespace romea

#endif
