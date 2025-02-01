/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/tasks/speed_bounds_decider/speed_limit_decider.h"

#include <algorithm>
#include <limits>
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"
#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

SpeedLimitDecider::SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                                     const ReferenceLine& reference_line,
                                     const PathData& path_data)
    : speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
}

// 提取速度限制
Status SpeedLimitDecider::GetSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  // 提取之前生成的sl路径
  const auto& discretized_path = path_data_.discretized_path();
  const auto& frenet_path = path_data_.frenet_frame_path();
  PrintCurves print_curve;

  // 遍历离散点
  for (uint32_t i = 0; i < discretized_path.size(); ++i) {
    const double path_s = discretized_path.at(i).s();
    const double reference_line_s = frenet_path.at(i).s();

    // 忽略参考线之外的点
    if (reference_line_s > reference_line_.Length()) {
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_.Length() << "]. Please debug before proceeding.";
      break;
    }

    // 从地图/参考线上获取速度限制
    // (1) speed limit from map
    double speed_limit_from_reference_line =
        reference_line_.GetSpeedLimitFromS(reference_line_s);
    // print_curve.AddPoint("speed_limit_from_ref", path_s,
    //                      speed_limit_from_reference_line);
    // (2) speed limit from path curvature
    //  -- 2.1: limit by centripetal force (acceleration)
    // 根据曲率计算离心最大速度限制
    const double speed_limit_from_centripetal_acc =
        std::sqrt(speed_bounds_config_.max_centric_acceleration_limit() /
                  std::fmax(std::fabs(discretized_path.at(i).kappa()),
                            speed_bounds_config_.minimal_kappa()));
    // print_curve.AddPoint("speed_limit_from_centripetal_acc", path_s,
    //                      speed_limit_from_centripetal_acc);
    // (3) speed limit from nudge obstacles
    // TODO(all): in future, expand the speed limit not only to obstacles with
    // nudge decisions.
    double speed_limit_from_nearby_obstacles =
        std::numeric_limits<double>::max();
    const double collision_safety_range =
        speed_bounds_config_.collision_safety_range();
    // 遍历所有障碍物
    for (const auto* ptr_obstacle : obstacles.Items()) {
      // 忽略虚拟障碍物
      if (ptr_obstacle->IsVirtual()) {
        continue;
      }
      // 忽略非绕行障碍物
      if (!ptr_obstacle->LateralDecision().has_nudge()) {
        continue;
      }

      /* ref line:
       * -------------------------------
       *    start_s   end_s
       * ------|  adc   |---------------
       * ------------|  obstacle |------
       */

      // TODO(all): potential problem here;
      // frenet and cartesian coordinates are mixed.

      // 提取自车前后s
      const double vehicle_front_s =
          reference_line_s + vehicle_param_.front_edge_to_center();
      const double vehicle_back_s =
          reference_line_s - vehicle_param_.back_edge_to_center();

      // 提取障碍物前后s
      const double obstacle_front_s =
          ptr_obstacle->PerceptionSLBoundary().end_s();
      const double obstacle_back_s =
          ptr_obstacle->PerceptionSLBoundary().start_s();

      // 如果在s上没有交集，那么也可以忽略
      if (vehicle_front_s < obstacle_back_s ||
          vehicle_back_s > obstacle_front_s) {
        continue;
      }

      const auto& nudge_decision = ptr_obstacle->LateralDecision().nudge();

      // Please notice the differences between adc_l and frenet_point_l
      const double frenet_point_l = frenet_path.at(i).l();

      // obstacle is on the right of ego vehicle (at path point i)
      // 判断障碍物是否在右侧
      bool is_close_on_left =
          (nudge_decision.type() == ObjectNudge::LEFT_NUDGE) &&
          (frenet_point_l - vehicle_param_.right_edge_to_center() -
               collision_safety_range <
           ptr_obstacle->PerceptionSLBoundary().end_l());

      // 判断障碍物是否在左侧
      // obstacle is on the left of ego vehicle (at path point i)
      bool is_close_on_right =
          (nudge_decision.type() == ObjectNudge::RIGHT_NUDGE) &&
          (ptr_obstacle->PerceptionSLBoundary().start_l() -
               collision_safety_range <
           frenet_point_l + vehicle_param_.left_edge_to_center());

      // 
      // TODO(all): dynamic obstacles do not have nudge decision
      // 如果在左侧或者右侧，需要进一步限速
      if (is_close_on_left || is_close_on_right) {
        double nudge_speed_ratio = 1.0;
        if (ptr_obstacle->IsStatic()) {
          nudge_speed_ratio =
              speed_bounds_config_.static_obs_nudge_speed_ratio(); // 0.6
        } else {
          nudge_speed_ratio =
              speed_bounds_config_.dynamic_obs_nudge_speed_ratio(); // 0.8
        }
        speed_limit_from_nearby_obstacles =
            nudge_speed_ratio * speed_limit_from_reference_line;
        break;
      }
    }

    // 根据已有配置进一步限速
    double curr_speed_limit = 0.0;
    if (speed_bounds_config_.enable_nudge_slowdown()) {
      curr_speed_limit =
          std::fmax(speed_bounds_config_.lowest_speed(),
                    std::min({speed_limit_from_reference_line,
                              speed_limit_from_centripetal_acc,
                              speed_limit_from_nearby_obstacles}));
    } else {
      curr_speed_limit =
          std::fmax(speed_bounds_config_.lowest_speed(),
                    std::min({speed_limit_from_reference_line,
                              speed_limit_from_centripetal_acc}));
    }
    // if (speed_limit_from_nearby_obstacles <
        // std::numeric_limits<double>::max()) {
      // print_curve.AddPoint("speed_limit_from_nearby_obstacles", path_s,
      //                      speed_limit_from_nearby_obstacles);
    // }

    // 记录当前限速
    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
    // print_curve.AddPoint("curr_speed_limit", path_s, curr_speed_limit);
  }
  // print_curve.PrintToLog();
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
