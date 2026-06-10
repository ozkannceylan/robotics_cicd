// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "otonav_nav/go_to_goal.hpp"

using otonav_nav::go_to_goal;
using otonav_nav::GoToGoalParams;
using otonav_nav::VelocityCommand;
using otonav_nav::wrap_angle;

namespace {
const GoToGoalParams kParams;  // defaults
}  // namespace

TEST(GoToGoal, DrivesForwardWhenFacingGoal) {
  const VelocityCommand c = go_to_goal(1.0, 0.0, 0.0, kParams);
  EXPECT_FALSE(c.reached);
  EXPECT_GT(c.v, 0.0);
  EXPECT_NEAR(c.w, 0.0, 1e-9);
}

TEST(GoToGoal, TurnsLeftForGoalOnTheLeft) {
  const VelocityCommand c = go_to_goal(0.0, 1.0, 0.0, kParams);
  EXPECT_GT(c.w, 0.0);  // CCW toward +y
}

TEST(GoToGoal, ReachedWithinTolerance) {
  const VelocityCommand c = go_to_goal(0.05, 0.0, 0.0, kParams);  // tol = 0.1
  EXPECT_TRUE(c.reached);
  EXPECT_NEAR(c.v, 0.0, 1e-9);
  EXPECT_NEAR(c.w, 0.0, 1e-9);
}

TEST(GoToGoal, DoesNotDriveForwardWhenFacingAway) {
  const VelocityCommand c = go_to_goal(1.0, 0.0, M_PI, kParams);
  EXPECT_NEAR(c.v, 0.0, 1e-9);
  EXPECT_GT(std::abs(c.w), 0.0);
}

TEST(GoToGoal, ClampsToLimits) {
  GoToGoalParams p;
  p.v_max = 0.5;
  p.w_max = 1.0;
  const VelocityCommand c = go_to_goal(100.0, 0.0, -M_PI_2, p);  // far + big heading error
  EXPECT_LE(c.v, p.v_max + 1e-9);
  EXPECT_LE(std::abs(c.w), p.w_max + 1e-9);
}

TEST(GoToGoal, WrapAngleRange) {
  EXPECT_NEAR(wrap_angle(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(wrap_angle(-3.0 * M_PI), -M_PI, 1e-9);
  EXPECT_NEAR(wrap_angle(0.5), 0.5, 1e-9);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
