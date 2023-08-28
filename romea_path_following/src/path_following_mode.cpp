//romea
#include "romea_path_following/path_following_mode.hpp"

//std
#include <limits>

namespace romea
{

//-----------------------------------------------------------------------------
PathFollowingMode deducePathFollowingMode(
  const romea_path_msgs::msg::PathMatchedPoint2D & matched_point_msg)
{
  if (matched_point_msg.section_index != std::numeric_limits<size_t>::max()) {
    return PathFollowingMode::CLASSIC_PATH_FOLLOWING;
  } else {
    if (matched_point_msg.curve_index != std::numeric_limits<size_t>::max()) {
      return PathFollowingMode::PLATOON_PATH_FOLLOWING;
    } else {
      return PathFollowingMode::ON_APPROACH;
    }
  }
}

//-----------------------------------------------------------------------------
std::string toString(const PathFollowingMode & status)
{
  switch (status) {
    case PathFollowingMode::CLASSIC_PATH_FOLLOWING:
      return "Classic path matching";
    case PathFollowingMode::PLATOON_PATH_FOLLOWING:
      return "On the fly path matching";
    case PathFollowingMode::ON_APPROACH:
      return "Fake path matching";
    default:
      return "Unknown";
  }
}

}  //End of namespace romea
