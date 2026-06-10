// Copyright 2026 Ozkan Ceylan
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "otonav_control/diff_drive_kinematics.hpp"

using otonav_control::BodyTwist;
using otonav_control::forward_kinematics;
using otonav_control::inverse_kinematics;
using otonav_control::WheelSpeeds;

namespace {
constexpr double kRadius = 0.05;
constexpr double kTrack = 0.30;
constexpr double kEps = 1e-9;
}  // namespace

TEST(DiffDriveKinematics, ForwardInverseRoundTrip) {
  for (double v : {-1.0, -0.25, 0.0, 0.3, 1.5}) {
    for (double w : {-2.0, -0.5, 0.0, 0.7, 3.0}) {
      const BodyTwist in{v, w};
      const WheelSpeeds wheels = inverse_kinematics(in, kRadius, kTrack);
      const BodyTwist out = forward_kinematics(wheels, kRadius, kTrack);
      EXPECT_NEAR(in.v, out.v, kEps) << "v mismatch at v=" << v << " w=" << w;
      EXPECT_NEAR(in.w, out.w, kEps) << "w mismatch at v=" << v << " w=" << w;
    }
  }
}

TEST(DiffDriveKinematics, PureRotationWheelsOppose) {
  const WheelSpeeds w = inverse_kinematics(BodyTwist{0.0, 1.0}, kRadius, kTrack);
  EXPECT_NEAR(w.left, -w.right, kEps);
  EXPECT_LT(w.left, 0.0);  // spin left wheel back, right wheel forward -> CCW
  EXPECT_GT(w.right, 0.0);
}

TEST(DiffDriveKinematics, PureTranslationWheelsEqual) {
  const WheelSpeeds w = inverse_kinematics(BodyTwist{0.5, 0.0}, kRadius, kTrack);
  EXPECT_NEAR(w.left, w.right, kEps);
  EXPECT_NEAR(w.left, 0.5 / kRadius, kEps);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
