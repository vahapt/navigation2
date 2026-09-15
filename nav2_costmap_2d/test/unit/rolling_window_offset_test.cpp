// Copyright (c) 2026 Vahap Tokmak
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{

void setPose(tf2::BufferCore & tf, double t, double x, double y, double yaw)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp.sec = static_cast<int32_t>(std::floor(t));
  transform.header.stamp.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  tf.setTransform(transform, "test");
}

}  // namespace

TEST(RollingWindowOffset, velocityInGlobalFrame)
{
  tf2::BufferCore tf;
  // Robot faces +y and drives forward at 1 m/s, so its global velocity is along y
  for (int i = 0; i <= 10; ++i) {
    double t = 1.0 + 0.1 * i;
    setPose(tf, t, 2.0, 0.1 * i, M_PI_2);
  }

  double offset_x, offset_y;
  ASSERT_TRUE(
    nav2_costmap_2d::computeRollingWindowOffset(
      tf, "odom", "base_link", 0.5, 2.0, 3.0, 3.0, offset_x, offset_y));
  EXPECT_NEAR(offset_x, 0.0, 1e-3);
  EXPECT_NEAR(offset_y, 2.0, 1e-3);
}

TEST(RollingWindowOffset, clampedToMaxOffset)
{
  tf2::BufferCore tf;
  // Robot drives backward along -x at 2 m/s
  for (int i = 0; i <= 10; ++i) {
    double t = 1.0 + 0.1 * i;
    setPose(tf, t, -0.2 * i, 0.0, 0.0);
  }

  double offset_x, offset_y;
  ASSERT_TRUE(
    nav2_costmap_2d::computeRollingWindowOffset(
      tf, "odom", "base_link", 0.5, 2.0, 1.5, 1.5, offset_x, offset_y));
  EXPECT_NEAR(offset_x, -1.5, 1e-3);
  EXPECT_NEAR(offset_y, 0.0, 1e-3);
}

TEST(RollingWindowOffset, missingTransform)
{
  tf2::BufferCore tf;
  double offset_x = 1.0, offset_y = 1.0;
  EXPECT_FALSE(
    nav2_costmap_2d::computeRollingWindowOffset(
      tf, "odom", "base_link", 0.5, 2.0, 1.5, 1.5, offset_x, offset_y));
  EXPECT_DOUBLE_EQ(offset_x, 0.0);
  EXPECT_DOUBLE_EQ(offset_y, 0.0);
}
