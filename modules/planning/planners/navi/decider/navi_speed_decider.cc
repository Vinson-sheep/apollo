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
 * @file
 * @brief This file provides the implementation of the class "NaviSpeedDecider".
 */

#include "modules/planning/planners/navi/decider/navi_speed_decider.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Clamp;

namespace {
constexpr double kTsGraphSStep = 0.4;
constexpr size_t kFallbackSpeedPointNum = 4;
constexpr double kSpeedPointSLimit = 200.0;
constexpr double kSpeedPointTimeLimit = 50.0;
constexpr double kZeroSpeedEpsilon = 1.0e-3;
constexpr double kZeroAccelEpsilon = 1.0e-3;
constexpr double kDecelCompensationLimit = 2.0;
constexpr double kKappaAdjustRatio = 20.0;
}  // namespace

NaviSpeedDecider::NaviSpeedDecider() : NaviTask("NaviSpeedDecider") {}

bool NaviSpeedDecider::Init(const PlannerNaviConfig& config) {

  // 安全校验
  CHECK_GT(FLAGS_planning_upper_speed_limit, 0.0);
  ACHECK(config.has_navi_speed_decider_config());
  ACHECK(config.navi_speed_decider_config().has_preferred_accel());
  ACHECK(config.navi_speed_decider_config().has_preferred_decel());
  ACHECK(config.navi_speed_decider_config().has_max_accel());
  ACHECK(config.navi_speed_decider_config().has_max_decel());
  ACHECK(config.navi_speed_decider_config().has_preferred_jerk());
  ACHECK(config.navi_speed_decider_config().has_obstacle_buffer());
  ACHECK(config.navi_speed_decider_config().has_safe_distance_base());
  ACHECK(config.navi_speed_decider_config().has_safe_distance_ratio());
  ACHECK(config.navi_speed_decider_config().has_following_accel_ratio());
  ACHECK(config.navi_speed_decider_config().has_soft_centric_accel_limit());
  ACHECK(config.navi_speed_decider_config().has_hard_centric_accel_limit());
  ACHECK(config.navi_speed_decider_config().has_hard_speed_limit());
  ACHECK(config.navi_speed_decider_config().has_hard_accel_limit());
  ACHECK(config.navi_speed_decider_config().has_enable_safe_path());
  ACHECK(config.navi_speed_decider_config().has_enable_planning_start_point());
  ACHECK(
      config.navi_speed_decider_config().has_enable_accel_auto_compensation());
  ACHECK(config.navi_speed_decider_config().has_kappa_preview());
  ACHECK(config.navi_speed_decider_config().has_kappa_threshold());

  // 加载参数
  max_speed_ = FLAGS_planning_upper_speed_limit;
  preferred_accel_ =
      std::abs(config.navi_speed_decider_config().preferred_accel());
  preferred_decel_ =
      std::abs(config.navi_speed_decider_config().preferred_decel());
  preferred_jerk_ =
      std::abs(config.navi_speed_decider_config().preferred_jerk());
  max_accel_ = std::abs(config.navi_speed_decider_config().max_accel());
  max_decel_ = std::abs(config.navi_speed_decider_config().max_decel());
  preferred_accel_ = std::min(max_accel_, preferred_accel_);
  preferred_decel_ = std::min(max_decel_, preferred_accel_);

  obstacle_buffer_ =
      std::abs(config.navi_speed_decider_config().obstacle_buffer());
  safe_distance_base_ =
      std::abs(config.navi_speed_decider_config().safe_distance_base());
  safe_distance_ratio_ =
      std::abs(config.navi_speed_decider_config().safe_distance_ratio());
  following_accel_ratio_ =
      std::abs(config.navi_speed_decider_config().following_accel_ratio());
  soft_centric_accel_limit_ =
      std::abs(config.navi_speed_decider_config().soft_centric_accel_limit());
  hard_centric_accel_limit_ =
      std::abs(config.navi_speed_decider_config().hard_centric_accel_limit());
  soft_centric_accel_limit_ =
      std::min(hard_centric_accel_limit_, soft_centric_accel_limit_);
  hard_speed_limit_ =
      std::abs(config.navi_speed_decider_config().hard_speed_limit());
  hard_accel_limit_ =
      std::abs(config.navi_speed_decider_config().hard_accel_limit());
  enable_safe_path_ = config.navi_speed_decider_config().enable_safe_path();
  enable_planning_start_point_ =
      config.navi_speed_decider_config().enable_planning_start_point();
  enable_accel_auto_compensation_ =
      config.navi_speed_decider_config().enable_accel_auto_compensation();
  kappa_preview_ = config.navi_speed_decider_config().kappa_preview();
  kappa_threshold_ = config.navi_speed_decider_config().kappa_threshold();

  return obstacle_decider_.Init(config);
}

Status NaviSpeedDecider::Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info) {
  NaviTask::Execute(frame, reference_line_info);

  // 计算巡航速度
  const auto& planning_target = reference_line_info_->planning_target();
  preferred_speed_ = planning_target.has_cruise_speed()
                         ? std::abs(planning_target.cruise_speed())
                         : 0.0;
  preferred_speed_ = std::min(max_speed_, preferred_speed_);

  // 计算规划初始状态
  const auto& planning_start_point = frame->PlanningStartPoint();
  auto expected_v =
      planning_start_point.has_v() ? planning_start_point.v() : 0.0;
  auto expected_a =
      planning_start_point.has_a() ? planning_start_point.a() : 0.0;

  // 计算自车当前状态
  const auto& vehicle_state = frame->vehicle_state();
  auto current_v = vehicle_state.has_linear_velocity()
                       ? vehicle_state.linear_velocity()
                       : 0.0;
  auto current_a = vehicle_state.has_linear_acceleration()
                       ? vehicle_state.linear_acceleration()
                       : 0.0;

  // 获取实际初始规划状态
  double start_v;
  double start_a;
  double start_da;

  if (enable_planning_start_point_) {
    start_v = std::max(0.0, expected_v);
    start_a = expected_a;
    start_da = 0.0;
  } else {
    start_v = std::max(0.0, current_v);
    start_a = current_a;
    start_da = 0.0;
  }

  // 估算减速补偿比？
  if (enable_accel_auto_compensation_) {
    if (prev_v_ > 0.0 && current_v > 0.0 && prev_v_ > expected_v) {
      auto raw_ratio = (prev_v_ - expected_v) / (prev_v_ - current_v);
      raw_ratio = Clamp(raw_ratio, 0.0, kDecelCompensationLimit);
      decel_compensation_ratio_ = (decel_compensation_ratio_ + raw_ratio) / 2.0;
      decel_compensation_ratio_ =
          Clamp(decel_compensation_ratio_, 1.0, kDecelCompensationLimit);
      // ADEBUG << "change decel_compensation_ratio: " << decel_compensation_ratio_
      //        << " raw: " << raw_ratio;
    }
    prev_v_ = current_v;
  }

  // 获取path_planner参考路径
  auto& discretized_path = reference_line_info_->path_data().discretized_path();

  // 进行速度决策
  auto ret = MakeSpeedDecision(
      start_v, start_a, start_da, discretized_path, frame_->obstacles(),
      [&](const std::string& id) { return frame_->Find(id); },
      reference_line_info_->mutable_speed_data());

  // RecordDebugInfo(reference_line_info->speed_data());

  // 后处理
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }

  return ret;
}

Status NaviSpeedDecider::MakeSpeedDecision(
    double start_v, double start_a, double start_da,
    const std::vector<PathPoint>& path_points,
    const std::vector<const Obstacle*>& obstacles,
    const std::function<const Obstacle*(const std::string&)>& find_obstacle,
    SpeedData* const speed_data) {
  CHECK_NOTNULL(speed_data);
  CHECK_GE(path_points.size(), 2U);

  // 获取初始s和末端s
  auto start_s = path_points.front().has_s() ? path_points.front().s() : 0.0;
  auto end_s = path_points.back().has_s() ? path_points.back().s() : start_s;
  auto planning_length = end_s - start_s;

  // ADEBUG << "start to make speed decision,  start_v: " << start_v
  //        << " start_a: " << start_a << " start_da: " << start_da
  //        << " start_s: " << start_s << " planning_length: " << planning_length;

  // 如果初始速度异常，直接返回
  if (start_v > max_speed_) {
    const std::string msg = "exceeding maximum allowable speed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 初始速度必须大于0
  start_v = std::max(0.0, start_v);

  // 计算s分辨率
  auto s_step = planning_length > kTsGraphSStep
                    ? kTsGraphSStep
                    : planning_length / kFallbackSpeedPointNum;

  // 初始化st图
  ts_graph_.Reset(s_step, planning_length, start_v, start_a, start_da);

  // 增加感知范围限制 (未实现)
  auto ret = AddPerceptionRangeConstraints();
  if (ret != Status::OK()) {
    AERROR << "Add t-s constraints base on range of perception failed";
    return ret;
  }

  // 增加障碍物限制
  ret = AddObstaclesConstraints(start_v, planning_length, path_points,
                                obstacles, find_obstacle);
  if (ret != Status::OK()) {
    AERROR << "Add t-s constraints base on obstacles failed";
    return ret;
  }

  // 
  ret = AddCentricAccelerationConstraints(path_points);
  if (ret != Status::OK()) {
    AERROR << "Add t-s constraints base on centric acceleration failed";
    return ret;
  }

  // 配置边界约束
  ret = AddConfiguredConstraints();
  if (ret != Status::OK()) {
    AERROR << "Add t-s constraints base on configs failed";
    return ret;
  }

  // 基于st图进行速度规划
  std::vector<NaviSpeedTsPoint> ts_points;
  if (ts_graph_.Solve(&ts_points) != Status::OK()) {
    AERROR << "Solve speed points failed";
    speed_data->clear();
    speed_data->AppendSpeedPoint(0.0 + start_s, 0.0, 0.0, -max_decel_, 0.0);
    speed_data->AppendSpeedPoint(0.0 + start_s, 1.0, 0.0, -max_decel_, 0.0);
    return Status::OK();
  }

  speed_data->clear();

  // 遍历st点
  for (auto& ts_point : ts_points) {

    // 忽略范围外点
    if (ts_point.s > kSpeedPointSLimit || ts_point.t > kSpeedPointTimeLimit)
      break;

    // 修正v
    if (ts_point.v > hard_speed_limit_) {
      AERROR << "The v: " << ts_point.v << " of point with s: " << ts_point.s
             << " and t: " << ts_point.t << "is greater than hard_speed_limit "
             << hard_speed_limit_;
      ts_point.v = hard_speed_limit_;
    }

    // 接近0，直接置零
    if (std::abs(ts_point.v) < kZeroSpeedEpsilon) {
      ts_point.v = 0.0;
    }

    // 修正a
    if (ts_point.a > hard_accel_limit_) {
      AERROR << "The a: " << ts_point.a << " of point with s: " << ts_point.s
             << " and t: " << ts_point.t << "is greater than hard_accel_limit "
             << hard_accel_limit_;
      ts_point.a = hard_accel_limit_;
    }

    // 接近0，直接置零
    if (std::abs(ts_point.a) < kZeroAccelEpsilon) {
      ts_point.a = 0.0;
    }

    // 应用加速度补偿 (乘补偿因子)
    if (enable_accel_auto_compensation_) {
      if (ts_point.a > 0)
        ts_point.a *= accel_compensation_ratio_;
      else
        ts_point.a *= decel_compensation_ratio_;
    }

    speed_data->AppendSpeedPoint(ts_point.s + start_s, ts_point.t, ts_point.v,
                                 ts_point.a, ts_point.da);
  }

  if (speed_data->size() == 1) {
    const auto& prev = speed_data->back();
    speed_data->AppendSpeedPoint(prev.s(), prev.t() + 1.0, 0.0, 0.0, 0.0);
  }

  return Status::OK();
}

Status NaviSpeedDecider::AddPerceptionRangeConstraints() {
  // TODO(all):
  return Status::OK();
}

Status NaviSpeedDecider::AddObstaclesConstraints(
    double vehicle_speed, double path_length,
    const std::vector<PathPoint>& path_points,
    const std::vector<const Obstacle*>& obstacles,
    const std::function<const Obstacle*(const std::string&)>& find_obstacle) {
  // 获取自车参数
  const auto& vehicle_config = VehicleConfigHelper::Instance()->GetConfig();
  auto front_edge_to_center =
      vehicle_config.vehicle_param().front_edge_to_center();

  // 定义一些lambda
  auto get_obstacle_distance = [&](double d) -> double {
    return std::max(0.0, d - front_edge_to_center - obstacle_buffer_);
  };
  auto get_safe_distance = [&](double v) -> double {
    return safe_distance_ratio_ * v + safe_distance_base_ +
           front_edge_to_center + obstacle_buffer_;
  };

  // 提取路径附近的障碍物
  obstacle_decider_.GetUnsafeObstaclesInfo(path_points, obstacles);

  // 遍历所有相关障碍物
  for (const auto& info : obstacle_decider_.UnsafeObstacles()) {

    const auto& id = std::get<0>(info);

    // 提取障碍物
    const auto* obstacle = find_obstacle(id);
    if (obstacle != nullptr) {
      auto s = std::get<1>(info);

      // 计算障碍物距离
      auto obstacle_distance = get_obstacle_distance(s);

      // 计算障碍物速度
      auto obstacle_speed = std::max(std::get<2>(info), 0.0);
      
      // 计算安全距离
      auto safe_distance = get_safe_distance(obstacle_speed);
      // AINFO << "obstacle with id: " << id << " s: " << s
      //       << " distance: " << obstacle_distance
      //       << " speed: " << obstacle_speed
      //       << " safe_distance: " << safe_distance;

      // 将障碍物写入st图
      ts_graph_.UpdateObstacleConstraints(obstacle_distance, safe_distance,
                                          following_accel_ratio_,
                                          obstacle_speed, preferred_speed_);
    }
  }

  // 将终点当作虚拟障碍物
  if (enable_safe_path_) {
    auto obstacle_distance = get_obstacle_distance(path_length);
    auto safe_distance = get_safe_distance(0.0);
    ts_graph_.UpdateObstacleConstraints(obstacle_distance, safe_distance,
                                        following_accel_ratio_, 0.0,
                                        preferred_speed_);
  }

  return Status::OK();
}

Status AddTrafficDecisionConstraints() {
  // TODO(all):
  return Status::OK();
}

// 增加离心加速度约束
Status NaviSpeedDecider::AddCentricAccelerationConstraints(
    const std::vector<PathPoint>& path_points) {

  // 如果路径不合法，直接返回
  if (path_points.size() < 2) {
    const std::string msg = "Too few path points";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  double max_kappa = 0.0;
  double max_kappa_v = std::numeric_limits<double>::max();  // 最大速度
  double preffered_kappa_v = std::numeric_limits<double>::max();  // 推荐速度
  double max_kappa_s = 0.0; // 最大曲率对应的弧长位置
  const auto bs = path_points[0].has_s() ? path_points[0].s() : 0.0;  // 基准弧长

  struct CLimit {
    double s;
    double v_max;
    double v_preffered;
  };
  std::vector<CLimit> c_limits;
  c_limits.resize(path_points.size() - 1);

  for (size_t i = 1; i < path_points.size(); i++) {

    // 估算曲率
    const auto& prev = path_points[i - 1];
    const auto& cur = path_points[i];
    auto start_s = prev.has_s() ? prev.s() - bs : 0.0;
    start_s = std::max(0.0, start_s);
    auto end_s = cur.has_s() ? cur.s() - bs : 0.0;
    end_s = std::max(0.0, end_s);
    auto start_k = prev.has_kappa() ? prev.kappa() : 0.0;
    auto end_k = cur.has_kappa() ? cur.kappa() : 0.0;
    auto kappa = std::abs((start_k + end_k) / 2.0);
    if (std::abs(kappa) < kappa_threshold_) {
      kappa /= kKappaAdjustRatio;
    }

    // 估算最大速度
    auto v_preffered = std::min(std::sqrt(soft_centric_accel_limit_ / kappa),
                                std::numeric_limits<double>::max());
    auto v_max = std::min(std::sqrt(hard_centric_accel_limit_ / kappa),
                          std::numeric_limits<double>::max());

    c_limits[i - 1].s = end_s;
    c_limits[i - 1].v_max = v_max;
    c_limits[i - 1].v_preffered = v_preffered;

    if (kappa > max_kappa) {
      max_kappa = kappa;
      max_kappa_v = v_max;
      preffered_kappa_v = v_preffered;
      max_kappa_s = start_s;
    }
  }

  // kappa preview
  for (size_t i = 0; i < c_limits.size(); i++) {
    for (size_t j = i; j - i < (size_t)(kappa_preview_ / kTsGraphSStep) &&
                       j < c_limits.size();
         j++)
      c_limits[i].v_preffered =
          std::min(c_limits[j].v_preffered, c_limits[i].v_preffered);
  }

  double start_s = 0.0;

  // 将速度约束插入到st图
  for (size_t i = 0; i < c_limits.size(); i++) {
    auto end_s = c_limits[i].s;
    auto v_max = c_limits[i].v_max;
    auto v_preffered = c_limits[i].v_preffered;

    NaviSpeedTsConstraints constraints;
    constraints.v_max = v_max;
    constraints.v_preffered = v_preffered;
    ts_graph_.UpdateRangeConstraints(start_s, end_s, constraints);

    start_s = end_s;
  }

  AINFO << "add speed limit for centric acceleration with kappa: " << max_kappa
        << " v_max: " << max_kappa_v << " v_preffered: " << preffered_kappa_v
        << " s: " << max_kappa_s;

  return Status::OK();
}

Status NaviSpeedDecider::AddConfiguredConstraints() {
  NaviSpeedTsConstraints constraints;
  constraints.v_max = max_speed_;
  constraints.v_preffered = preferred_speed_;
  constraints.a_max = max_accel_;
  constraints.a_preffered = preferred_accel_;
  constraints.b_max = max_decel_;
  constraints.b_preffered = preferred_decel_;
  constraints.da_preffered = preferred_jerk_;
  ts_graph_.UpdateConstraints(constraints);

  return Status::OK();
}

void NaviSpeedDecider::RecordDebugInfo(const SpeedData& speed_data) {
  auto* debug = reference_line_info_->mutable_debug();
  auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
  ptr_speed_plan->set_name(Name());
  ptr_speed_plan->mutable_speed_point()->CopyFrom(
      {speed_data.begin(), speed_data.end()});
}

}  // namespace planning
}  // namespace apollo
