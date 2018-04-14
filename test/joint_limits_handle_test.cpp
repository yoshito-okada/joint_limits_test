#include <gtest/gtest.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "range_based_handle.h"

template < class VelocitySaturationHandle > void testVelocitySaturationSymmetry() {
  // General joint handle
  double pos(0.), vel(0.), eff(0.), cmd(0.);
  hardware_interface::JointStateHandle state_handle("joint", &pos, &vel, &eff);
  hardware_interface::JointHandle command_handle(state_handle, &cmd);

  // Condition: velocity & acceleration limits are active
  joint_limits_interface::JointLimits limits;
  limits.has_velocity_limits = true;
  limits.max_velocity = 5.;
  limits.has_acceleration_limits = true;
  limits.max_acceleration = 20.;

  // Velocity command saturation handle to be tested
  VelocitySaturationHandle saturation_handle(command_handle, limits);

  //
  // Case 1: velocity is greater than upper velocity limit
  //     this is not a special case when external force acts the joint
  //     (ex. collaborative robot touched by human).
  //
  const double vel_case1(10.);
  vel = vel_case1;

  // Velocity command saturation
  cmd = 0.;
  saturation_handle.enforceLimits(ros::Duration(0.1));
  const double cmd_case1(cmd);

  //
  // Case 2: velocity is inverted
  //
  vel = -vel_case1;

  // Velocity command saturation
  cmd = 0.;
  saturation_handle.enforceLimits(ros::Duration(0.1));
  const double cmd_case2(cmd);

  // Expectation: inverted saturation
  EXPECT_TRUE(cmd_case1 == -cmd_case2);

  ROS_INFO_STREAM("\n"
                  << "Saturated command\n"
                  << "    case 1: " << cmd_case1 << "\n"
                  << "    case 2: " << cmd_case2);
}

// Test for current implementation
TEST(VelocitySaturationSymmetry, currentSaturation) {
  testVelocitySaturationSymmetry< joint_limits_interface::VelocityJointSaturationHandle >();
}

// Test for proposed range-based implementation
TEST(VelocitySaturationSymmetry, rangeBasedSaturation) {
  testVelocitySaturationSymmetry< range_based::VelocityJointSaturationHandle >();
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}