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

#include "modules/planning/tasks/speed_bounds_decider/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

STBoundaryMapper::STBoundaryMapper(
    const SpeedBoundsDeciderConfig& config, const ReferenceLine& reference_line,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    const std::shared_ptr<DependencyInjector>& injector)
    : speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      planning_max_distance_(planning_distance),
      planning_max_time_(planning_time),
      injector_(injector) {}

// path_decision 存放了所有已经做出决策的障碍物
Status STBoundaryMapper::ComputeSTBoundary(PathDecision* path_decision) const {
  // Sanity checks.
  CHECK_GT(planning_max_time_, 0.0);
  // 如果路径点小于两个，不合法
  if (path_data_.discretized_path().size() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().size() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  // Go through every obstacle.
  Obstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();

  // 遍历所有障碍物
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {

    // 基于ID提取障碍物
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    ACHECK(ptr_obstacle != nullptr);

    // If no longitudinal decision has been made, then plot it onto ST-graph.

    // 如果障碍物没有纵向决策，根据预测轨迹绘制障碍物st图
    if (!ptr_obstacle->HasLongitudinalDecision()) {
      ComputeSTBoundary(ptr_obstacle);
      continue;
    }

    // 
    // If there is a longitudinal decision, then fine-tune boundary.
    const auto& decision = ptr_obstacle->LongitudinalDecision();

    // 如果障碍物纵向决策为停止，按照静态障碍物生成st图
    if (decision.has_stop()) {
      // 1. Store the closest stop fence info.
      // TODO(all): store ref. s value in stop decision; refine the code then.
      common::SLPoint stop_sl_point;
      reference_line_.XYToSL(decision.stop().stop_point(), &stop_sl_point);
      const double stop_s = stop_sl_point.s();

      if (stop_s < min_stop_s) {
        stop_obstacle = ptr_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } 
    // 否则，如果有其他纵向决策，那么依据决策类型生成st图
    else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      // 2. Depending on the longitudinal overtake/yield decision,
      //    fine-tune the upper/lower st-boundary of related obstacles.
      ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
    } 
    // 如果不可以忽略，那么报错
    else if (!decision.has_ignore()) {
      // 3. Ignore those unrelated obstacles.
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }

  // 按照静态障碍物生成st图
  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      const std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  return Status::OK();
}

// 对静态障碍物生成st图
bool STBoundaryMapper::MapStopDecision(
    Obstacle* stop_obstacle, const ObjectDecisionType& stop_decision) const {
  
  // 如果不是静态障碍物，报错
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";

  // 将停止点转换为sl坐标
  common::SLPoint stop_sl_point;
  reference_line_.XYToSL(stop_decision.stop().stop_point(), &stop_sl_point);

  double st_stop_s = 0.0;
  const double stop_ref_s =
      stop_sl_point.s() - vehicle_param_.front_edge_to_center();

  // 获取停止点在参考线的投影点
  if (stop_ref_s > path_data_.frenet_frame_path().back().s()) {
    st_stop_s = path_data_.discretized_path().back().s() +
                (stop_ref_s - path_data_.frenet_frame_path().back().s());
  } else {
    PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      return false;
    }
    st_stop_s = stop_point.s();
  }

  // 计算s的区间
  const double s_min = std::fmax(0.0, st_stop_s);
  const double s_max = std::fmax(
      s_min, std::fmax(planning_max_distance_, reference_line_.Length()));

  // 计算st图
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_max_time_),
      STPoint(s_max + speed_bounds_config_.boundary_buffer(),
              planning_max_time_));
  auto boundary = STBoundary(point_pairs);
  boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(speed_bounds_config_.boundary_buffer());
  boundary.set_id(stop_obstacle->Id());
  stop_obstacle->set_path_st_boundary(boundary);
  return true;
}

void STBoundaryMapper::ComputeSTBoundary(Obstacle* obstacle) const {
  if (FLAGS_use_st_drivable_boundary) {
    return;
  }
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  // 遍历所有障碍物，获取障碍物与路径的重叠区域
  if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                &upper_points, &lower_points)) {
    return;
  }

  // 将重叠区域实例化
  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  boundary.set_id(obstacle->Id());

  // TODO(all): potential bug here.
  // 如果障碍物之前存在st图，那么沿用st图类型
  // 否则，使用新st图类型
  const auto& prev_st_boundary = obstacle->path_st_boundary();
  const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }

  obstacle->set_path_st_boundary(boundary);
}

// 遍历所有障碍物，获取与路径重叠的边界
bool STBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  // Sanity checks.
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());

  // 如果路径点为空，则返回
  if (path_points.empty()) {
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }

  const auto* planning_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_change_lane();

  double l_buffer =
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE
          ? speed_bounds_config_.lane_change_obstacle_nudge_l_buffer()
          : FLAGS_nonstatic_obstacle_nudge_l_buffer; // 横向裕度

  // 提取障碍物轨迹和尺寸
  // Draw the given obstacle on the ST-graph.
  const auto& trajectory = obstacle.Trajectory();
  const double obstacle_length = obstacle.Perception().length();
  const double obstacle_width = obstacle.Perception().width();

  // 如果障碍物轨迹为空
  if (trajectory.trajectory_point().empty()) {
    bool box_check_collision = false;

    // For those with no predicted trajectories, just map the obstacle's
    // current position to ST-graph and always assume it's static.
    // 轨迹为空一般是静态障碍物，如果不是，那么肯定有问题
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }

    const Box2d& obs_box = obstacle.PerceptionBoundingBox();

    // 遍历所有路径点
    for (const auto& curr_point_on_path : path_points) {
      // 忽略超出阈值的路径点
      if (curr_point_on_path.s() > planning_max_distance_) {
        break;
      }
      // 校验碰撞
      if (CheckOverlap(curr_point_on_path, obs_box, l_buffer)) {
        box_check_collision = true;
        break;
      }
    }

    // 如果轨迹与静态障碍物有碰撞
    if (box_check_collision) {
      // const double backward_distance =
      // -vehicle_param_.front_edge_to_center();
      const double backward_distance = 0;
      const double forward_distance = obs_box.length();

      for (const auto& curr_point_on_path : path_points) {
        if (curr_point_on_path.s() > planning_max_distance_) {
          break;
        }
        const Polygon2d& obs_polygon = obstacle.PerceptionPolygon();
        if (CheckOverlap(curr_point_on_path, obs_polygon, l_buffer)) {
          // If there is overlapping, then plot it on ST-graph.
          double low_s =
              std::fmax(0.0, curr_point_on_path.s() + backward_distance);
          double high_s = std::fmin(planning_max_distance_,
                                    curr_point_on_path.s() + forward_distance);
          AINFO << "check colllision for obstacle[" << obstacle.Id()
                << "], at: " << curr_point_on_path.DebugString();
          // It is an unrotated rectangle appearing on the ST-graph.
          // TODO(jiacheng): reconsider the backward_distance, it might be
          // unnecessary, but forward_distance is indeed meaningful though.

          // speed_bounds_config_.point_extension() 就是一个膨胀裕度
          // 计算静态障碍物的st图
          lower_points->emplace_back(
              low_s - speed_bounds_config_.point_extension(), 0.0);
          lower_points->emplace_back(
              low_s - speed_bounds_config_.point_extension(),
              planning_max_time_);
          upper_points->emplace_back(
              high_s + speed_bounds_config_.point_extension(), 0.0);
          upper_points->emplace_back(
              high_s + speed_bounds_config_.point_extension(),
              planning_max_time_);
          break;
        }
      }
    }
  } 
  // 如果障碍物存在轨迹，大概是动态障碍物
  else {
    // For those with predicted trajectories (moving obstacles):
    // 1. Subsample to reduce computation time.
    // 对当前路径点进行抽稀
    const int default_num_point = 50;
    DiscretizedPath discretized_path;
    if (path_points.size() > 2 * default_num_point) {
      const auto ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path = DiscretizedPath(std::move(sampled_path_points));
    } else {
      discretized_path = DiscretizedPath(path_points);
    }

    // 2. Go through every point of the predicted obstacle trajectory.
    double trajectory_time_interval =
        obstacle.Trajectory().trajectory_point()[1].relative_time();
    int trajectory_step =
        std::min(FLAGS_trajectory_check_collision_time_step,
                 std::max(vehicle_param_.width() / obstacle.speed() /
                              trajectory_time_interval,
                          1.0));
    bool trajectory_point_collision_status = false;
    int previous_index = 0;

    // 遍历障碍物所有轨迹点
    for (int i = 0; i < trajectory.trajectory_point_size();
         i = std::min(i + trajectory_step,
                      trajectory.trajectory_point_size() - 1)) {

      // 提取单个轨迹点
      const auto& trajectory_point = trajectory.trajectory_point(i);

      // 提取障碍物形状
      Polygon2d obstacle_shape =
          obstacle.GetObstacleTrajectoryPolygon(trajectory_point);

      // 忽略某时间阈值之前的障碍物轨迹点
      double trajectory_point_time = trajectory_point.relative_time();
      static constexpr double kNegtiveTimeThreshold = -1.0;
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      // 从后往前遍历？为什么这么复杂
      bool collision = CheckOverlapWithTrajectoryPoint(
          discretized_path, obstacle_shape, upper_points, lower_points,
          l_buffer, default_num_point, obstacle_length, obstacle_width,
          trajectory_point_time);
      if ((trajectory_point_collision_status ^ collision) && i != 0) {
        // Start retracing track points forward
        int index = i - 1;
        while ((trajectory_point_collision_status ^ collision) &&
               index > previous_index) {
          const auto& point = trajectory.trajectory_point(index);
          trajectory_point_time = point.relative_time();
          obstacle_shape = obstacle.GetObstacleTrajectoryPolygon(point);
          collision = CheckOverlapWithTrajectoryPoint(
              discretized_path, obstacle_shape, upper_points, lower_points,
              l_buffer, default_num_point, obstacle_length, obstacle_width,
              trajectory_point_time);
          index--;
        }
        trajectory_point_collision_status = !trajectory_point_collision_status;
      }
      if (i == trajectory.trajectory_point_size() - 1) break;
      previous_index = i;
    }
  }

  // 按t进行排序
  // Sanity checks and return.
  std::sort(lower_points->begin(), lower_points->end(),
            [](const STPoint& a, const STPoint& b) { return a.t() < b.t(); });
  std::sort(upper_points->begin(), upper_points->end(),
            [](const STPoint& a, const STPoint& b) { return a.t() < b.t(); });
  DCHECK_EQ(lower_points->size(), upper_points->size());
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

bool STBoundaryMapper::CheckOverlapWithTrajectoryPoint(
    const DiscretizedPath& discretized_path, const Polygon2d& obstacle_shape,
    std::vector<STPoint>* upper_points, std::vector<STPoint>* lower_points,
    const double l_buffer, int default_num_point, const double obstacle_length,
    const double obstacle_width, const double trajectory_point_time) const {
  // 以车半长作为遍历步长
  const double step_length = vehicle_param_.front_edge_to_center();
  auto path_len = std::min(speed_bounds_config_.max_trajectory_len(),
                           discretized_path.Length());

  // 
  // Go through every point of the ADC's path.
  for (double path_s = 0.0; path_s < path_len; path_s += step_length) {

    // 估算当前路径点
    const auto curr_adc_path_point =
        discretized_path.Evaluate(path_s + discretized_path.front().s());

    // 如果路径当前点
    if (CheckOverlap(curr_adc_path_point, obstacle_shape, l_buffer)) {
      // Found overlap, start searching with higher resolution
      // const double backward_distance = -step_length;
      const double backward_distance = 0.0;
      const double forward_distance = vehicle_param_.length() +
                                      vehicle_param_.width() + obstacle_length +
                                      obstacle_width;
      const double default_min_step = 0.1;  // in meters
      const double fine_tuning_step_length = std::fmin(
          default_min_step, discretized_path.Length() / default_num_point);

      bool find_low = false;
      bool find_high = false;
      double low_s = std::fmax(0.0, path_s + backward_distance);
      double high_s =
          std::fmin(discretized_path.Length(), path_s + forward_distance);

      // Keep shrinking by the resolution bidirectionally until finally
      // locating the tight upper and lower bounds.
      while (low_s < high_s) {
        if (find_low && find_high) {
          break;
        }
        if (!find_low) {
          const auto& point_low =
              discretized_path.Evaluate(low_s + discretized_path.front().s());
          if (!CheckOverlap(point_low, obstacle_shape, l_buffer)) {
            low_s += fine_tuning_step_length;
          } else {
            find_low = true;
          }
        }
        if (!find_high) {
          const auto& point_high =
              discretized_path.Evaluate(high_s + discretized_path.front().s());
          if (!CheckOverlap(point_high, obstacle_shape, l_buffer)) {
            high_s -= fine_tuning_step_length;
          } else {
            find_high = true;
          }
        }
      }
      if (find_high && find_low) {
        lower_points->emplace_back(
            low_s - speed_bounds_config_.point_extension(),
            trajectory_point_time);
        upper_points->emplace_back(
            high_s + speed_bounds_config_.point_extension(),
            trajectory_point_time);
      }
      return true;
    }
  }
  return false;
}

void STBoundaryMapper::ComputeSTBoundaryWithDecision(
    Obstacle* obstacle, const ObjectDecisionType& decision) const {
  DCHECK(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  // 如果障碍物st图已经初始化，沿用旧st图
  if (FLAGS_use_st_drivable_boundary &&
      obstacle->is_path_st_boundary_initialized()) {
    const auto& path_st_boundary = obstacle->path_st_boundary();
    lower_points = path_st_boundary.lower_points();
    upper_points = path_st_boundary.upper_points();
  } 
  // 否则，按照预测轨迹生成st图
  else {
    if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points)) {
      return;
    }
  }


  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);

  // get characteristic_length and boundary_type.
  // 特殊纵向决策需要特殊的st图
  STBoundary::BoundaryType b_type = STBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  if (decision.has_follow()) {
    characteristic_length = std::fabs(decision.follow().distance_s());
    AINFO << "characteristic_length: " << characteristic_length;
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::FOLLOW;
  } else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = STBoundary::BoundaryType::OVERTAKE;
  } else {
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  boundary.SetBoundaryType(b_type);
  boundary.set_id(obstacle->Id());
  // 障碍物纵向决策带来特殊CharacteristicLength，用处未知
  boundary.SetCharacteristicLength(characteristic_length);  
  obstacle->set_path_st_boundary(boundary);
}

bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}

bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Polygon2d& obs_polygon,
                                    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC polygon overlaps with obstacle polygon.
  Polygon2d adc_polygon(adc_box);
  return obs_polygon.HasOverlap(adc_polygon);
}

}  // namespace planning
}  // namespace apollo
