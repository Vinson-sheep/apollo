/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file ego_info.h
 **/

#pragma once

#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common_msgs/routing_msgs/geometry.pb.h"

#include "cyber/common/macros.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_base/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

class EgoInfo {
 public:
  EgoInfo();

  ~EgoInfo() = default;

  bool Update(const common::TrajectoryPoint& start_point,
              const common::VehicleState& vehicle_state);

  void Clear();

  common::TrajectoryPoint start_point() const { return start_point_; }

  common::VehicleState vehicle_state() const { return vehicle_state_; }

  double front_clear_distance() const { return front_clear_distance_; }

  common::math::Box2d ego_box() const { return ego_box_; }

  void CalculateFrontObstacleClearDistance(
      const std::vector<const Obstacle*>& obstacles);

  void CalculateCurrentRouteInfo(
      const ReferenceLineProvider* reference_line_provider);

 private:
  FRIEND_TEST(EgoInfoTest, EgoInfoSimpleTest);

  void set_vehicle_state(const common::VehicleState& vehicle_state) {
    vehicle_state_ = vehicle_state;
  }

  void set_start_point(const common::TrajectoryPoint& start_point) {
    start_point_ = start_point;
    const auto& param = ego_vehicle_config_.vehicle_param();
    start_point_.set_a(
        std::fmax(std::fmin(start_point_.a(), param.max_acceleration()),
                  param.max_deceleration()));
  }

  void CalculateEgoBox(const common::VehicleState& vehicle_state);

  // stitched point (at stitching mode)
  // or real vehicle point (at non-stitching mode)
  // 起始点（在轨迹拼接模式下就是轨迹拼接点）或者车辆位置点（规划处于非轨迹拼接模式下）
  common::TrajectoryPoint start_point_;

  // ego vehicle state
  // 车辆状态
  common::VehicleState vehicle_state_;
  // 前方畅通距离，默认为300M
  double front_clear_distance_ = FLAGS_default_front_clear_distance;
  // 自车配置，主要包括车辆品牌，长宽高，轴距，最大加速度等车辆物理参数
  common::VehicleConfig ego_vehicle_config_;
  // 自车二维边界框，车辆航向方向为长度，其法向量为宽度
  common::math::Box2d ego_box_;

  apollo::hdmap::LaneWaypoint adc_waypoint_;
  double distance_to_destination_ = 0;
};

}  // namespace planning
}  // namespace apollo
