#ifndef RANGE_BASED_RANGE_BASED_HANDLE_H
#define RANGE_BASED_RANGE_BASED_HANDLE_H

#include <cmath>
#include <limits>

#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <ros/duration.h>

#include <boost/algorithm/clamp.hpp>

namespace range_based {

// Helper class to deal with range bounds
//   this would be more robust than current implementation
//   and make enforceLimits() functions more readable
class Range {
public:
  Range(const double bound0, const double bound1)
      : min_(std::min(bound0, bound1)), max_(std::max(bound0, bound1)) {}
  // initializers
  static Range entire() {
    return Range(-std::numeric_limits< double >::max(), std::numeric_limits< double >::max());
  }
  static Range positive() { // would be useful in EffortJointSaturationHandle
    return Range(0., std::numeric_limits< double >::max());
  }
  static Range negative() { return Range(-std::numeric_limits< double >::max(), 0.); }
  // getters
  double min() const { return min_; }
  double max() const { return max_; }
  // saturation
  double clamp(const double val) { return boost::algorithm::clamp(val, min_, max_); }
  Range clamp(const Range &range) { return Range(clamp(range.min_), clamp(range.max_)); }

private:
  double min_, max_;
};

class VelocityJointSaturationHandle {
public:
  VelocityJointSaturationHandle(const hardware_interface::JointHandle &jh,
                                const joint_limits_interface::JointLimits &limits)
      : jh_(jh), limits_(limits) {}

  void enforceLimits(const ros::Duration &period) {
    // Default: unlimited velocity bounds
    Range vel_range(Range::entire());
    // Overwrite bounds according to acc limits
    if (limits_.has_acceleration_limits) {
      const double vel(jh_.getVelocity());
      const double delta_vel(limits_.max_acceleration * period.toSec());
      vel_range = Range(vel - delta_vel, vel + delta_vel).clamp(vel_range);
    }
    // Overwrite bounds according to vel limits
    if (limits_.has_velocity_limits) {
      vel_range = Range(-limits_.max_velocity, limits_.max_velocity).clamp(vel_range);
    }
    // Clamp the command with bounds
    jh_.setCommand(vel_range.clamp(jh_.getCommand()));
  }

private:
  hardware_interface::JointHandle jh_;
  joint_limits_interface::JointLimits limits_;
};

} // namespace range_based

#endif