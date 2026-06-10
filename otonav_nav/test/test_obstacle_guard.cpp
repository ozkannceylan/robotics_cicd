// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "otonav_nav/obstacle_guard.hpp"

using otonav_nav::obstacle_ahead;

namespace {
// 13 beams over [-pi/2, pi/2], matching the MJCF fan.
constexpr double kAngleMin = -M_PI / 2.0;
constexpr double kInc = M_PI / 12.0;
constexpr double kFrontHalf = 0.35;  // ~20 deg
constexpr double kStop = 0.4;

std::vector<float> all_clear() { return std::vector<float>(13, 10.0F); }
}  // namespace

TEST(ObstacleGuard, ClearWhenAllFar) {
  EXPECT_FALSE(obstacle_ahead(all_clear(), kAngleMin, kInc, kFrontHalf, kStop));
}

TEST(ObstacleGuard, BlockedWhenBeamAheadIsClose) {
  std::vector<float> r = all_clear();
  r[6] = 0.25F;  // beam 6 is angle 0 (straight ahead)
  EXPECT_TRUE(obstacle_ahead(r, kAngleMin, kInc, kFrontHalf, kStop));
}

TEST(ObstacleGuard, IgnoresCloseBeamOutsideFrontSector) {
  std::vector<float> r = all_clear();
  r[0] = 0.1F;  // beam 0 is -90 deg, well outside the front sector
  EXPECT_FALSE(obstacle_ahead(r, kAngleMin, kInc, kFrontHalf, kStop));
}

TEST(ObstacleGuard, IgnoresNonPositiveAndNonFinite) {
  std::vector<float> r = all_clear();
  r[6] = -1.0F;  // no-hit sentinel
  EXPECT_FALSE(obstacle_ahead(r, kAngleMin, kInc, kFrontHalf, kStop));
  r[6] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(obstacle_ahead(r, kAngleMin, kInc, kFrontHalf, kStop));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
