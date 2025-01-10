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

#include "modules/planning/planning_base/common/trajectory_stitcher.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_model/vehicle_model.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleModel;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint point;
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_z(vehicle_state.z());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_relative_time(planning_cycle_time);
  return point;
}

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  // 如果当前车速小于0.1，且加速度小于0.4，则直接用当前车辆状态作为规划起始点
  // 起步阶段，速度加速度很小
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
    // 预测车辆位置
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);
  }

  return std::vector<TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new =
                      common::math::NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
// 在轨迹拼接中纳入第二种方法主要考虑的是连续帧之间的平滑性和连续性，因为轨迹的这两个特性会严重影响到
// 下游控制效果。理论上，如果控制跟踪和定位完美，不会出现任何误差，则完全可以使用方案1选择规划起始点，但是，
// 由于定位误差和控制误差/滞后原因，会导致每一规划时刻ADC位置大概率的偏离上一帧所规划的轨迹点，如果此时仍
// 采用方案1，会导致当前帧所规划轨迹和上一帧轨迹规划的不平滑和不连续，从而导致控制的抖动，也可能引起ADC行驶轨
// 迹发散，跟踪误差越来越大，偏离设定参考线。而方案2可以保证ADC附近

// vehicle_state： 当前车辆状态 current_timestamp : 当前时间戳
// planning_cycle_time: 规划周期100ms preserved_points_num : 保留点数 20
// prev_trajectory : 上一帧轨迹 replan_reason：重规划原因
// p0=(t,x,y,v,a,heading,kappa)
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const canbus::Chassis& vehicle_chassis, const VehicleState& vehicle_state,
    const double current_timestamp, const double planning_cycle_time,
    const size_t preserved_points_num, const bool replan_by_offset,
    const PublishableTrajectory* prev_trajectory, std::string* replan_reason,
    const control::ControlInteractiveMsg& control_interactive_msg) {
  // 1.check replan not by offset
  // 第一种方法
  // 如果必须不使用轨迹拼接
  size_t time_matched_index = 0;
  if (need_replan_by_necessary_check(vehicle_state, current_timestamp,
                                     prev_trajectory, replan_reason,
                                     &time_matched_index)) {
    // 进入函数，主要利用自行车模型预测未来位置
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  // 第二种方法
  // 根据当前帧时间戳以及ADC实际位置信息（做比较），在上一帧轨迹中寻找匹配点，将上一帧轨迹中匹配点向前看0.1s
  // 所对应的轨迹点作为当前帧的规划起始状态点，待当前帧轨迹生成轨迹后，再和上一帧中所选择的轨迹
  // 起始点前一段距离的轨迹点进行拼接，形成一条完整的车辆运动轨迹，发送给下游控制模块
  // 2. check replan by GEAR switch
  if (vehicle_chassis.has_gear_location()) {
    static canbus::Chassis::GearPosition gear_pos =
        canbus::Chassis::GEAR_NEUTRAL;
    if (gear_pos == canbus::Chassis::GEAR_NEUTRAL &&
        vehicle_chassis.gear_location() == canbus::Chassis::GEAR_DRIVE) {
      gear_pos = vehicle_chassis.gear_location();
      /// 重规划原因：前后换档
      const std::string msg =
          "gear change from n to d, replan to avoid large station error";
      AERROR << msg;
      *replan_reason = msg;
      // 进入函数，主要利用自行车模型预测未来位置
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    gear_pos = vehicle_chassis.gear_location();
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));

  // 3.check replan by control_interactive_msg
  // 如果是控制交互触发的重规划
  if (need_replan_by_control_interactive(current_timestamp, replan_reason,
                                         control_interactive_msg)) {
    // 进入函数？？
    return ComputeControlInteractiveStitchingTrajectory(
        planning_cycle_time, vehicle_state, time_matched_point,
        control_interactive_msg);
  }

  // 根据当前车辆位置获取上一帧轨迹匹配点
  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

  // 计算实际位置和时间匹配点的横纵向误差
  // 根据车辆位置计算sd
  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));
  // replan_by_offset为true，判断横纵向偏差
  // 4.check replan by offset
  if (replan_by_offset) {
    if (vehicle_chassis.has_parking_brake()) {
      static bool parking_brake = true;
      if (parking_brake && !vehicle_chassis.parking_brake()) {
        parking_brake = vehicle_chassis.parking_brake();
        // 重规划原因：刹车
        const std::string msg =
            "parking brake off, ego move, replan to avoid large station error";
        AERROR << msg;
        *replan_reason = msg;
        // 进入函数，主要利用自行车模型预测未来位置
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
      }
      parking_brake = vehicle_chassis.parking_brake();
    }

    // 计算横纵向误差
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;
    double time_diff =
        time_matched_point.relative_time() -
        prev_trajectory
            ->TrajectoryPointAt(static_cast<uint32_t>(position_matched_index))
            .relative_time();

    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff
           << ", time diff: " << time_diff;
    // 如果纵向误差超出阈值
    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);
      AERROR << msg;
      // 重规划原因：纵向值超出阈值
      *replan_reason = msg;
      // 进入函数，主要利用自行车模型预测未来位置
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    if (std::fabs(time_diff) > FLAGS_replan_time_threshold) {
      const std::string msg = absl::StrCat(
          "the difference between time matched point relative time and "
          "actual position corresponding relative time is too "
          "large. Replan is triggered. time_diff = ",
          time_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
  } else {
    ADEBUG << "replan according to certain amount of "
           << "lat、lon and time offset is disabled";
  }

  // 4.stitching_trajectory

  // 以下为提取轨迹的核心内容
  // 当前时刻的绝对时间 - 上一时刻轨迹的初始时间 + planning_cycle_time
  // veh_rel_time为上一帧率自车未来行驶到的位置，这样当前帧自车的位置由了，再向前预测0.1s
  double forward_rel_time =
      current_timestamp - prev_trajectory->header_time() + planning_cycle_time;

  // 查询时间匹配点+0.1s
  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;

  // 获取时间和位置匹配点的最小索引值
  auto matched_index = std::min(time_matched_index, position_matched_index);
  // 根据preserved_points_num截取上一帧轨迹点
  // 从上一帧当前点位置向后看20个点，从上一帧率当前点位置向前看0.1s
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();
  // 获取拼接轨迹最后一个点的s值
  const double zero_s = stitching_trajectory.back().path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";、
      // 进入函数，主要利用自行车模型预测未来位置
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

bool TrajectoryStitcher::need_replan_by_necessary_check(
    const common::VehicleState& vehicle_state, const double current_timestamp,
    const PublishableTrajectory* prev_trajectory, std::string* replan_reason,
    size_t* time_matched_index) {
  // 配置文件不使用轨迹拼接
  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    return true;
  }
  // 如果没有上一帧轨迹
  if (!prev_trajectory) {
    *replan_reason = "replan for no previous trajectory.";
    return true;
  }
  // 如果当前驾驶模式不是自动驾驶
  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    return true;
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return true;
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  *time_matched_index = prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (*time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return true;
  }

  if (*time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return true;
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(*time_matched_index));

  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    return true;
  }
  return false;
}

bool TrajectoryStitcher::need_replan_by_control_interactive(
    const double current_timestamp, std::string* replan_reason,
    const control::ControlInteractiveMsg& control_interactive_msg) {
  // 上一次控制交互时间和当前时间差距过大
  const double rel_time =
      current_timestamp - control_interactive_msg.header().timestamp_sec();
  if (rel_time > 0.5) {
    AINFO << "control_interactive_msg time out, skip replay by control "
             "interactive";
    return false;
  }
  // 由控制交互发出的重规划请求
  if (control_interactive_msg.has_replan_request() &&
      control_interactive_msg.replan_request() == true) {
    *replan_reason = "replan for control_interactive_msg, " +
                     control_interactive_msg.replan_request_reason();
    return true;
  }
  return false;
}

std::vector<common::TrajectoryPoint>
TrajectoryStitcher::ComputeControlInteractiveStitchingTrajectory(
    const double planning_cycle_time, const common::VehicleState& vehicle_state,
    const common::TrajectoryPoint& time_match_point,
    const control::ControlInteractiveMsg& control_interactive_msg) {
  if (control_interactive_msg.replan_req_reason_code() ==
          control::ReplanRequestReasonCode::REPLAN_REQ_ALL_REPLAN ||
      control_interactive_msg.replan_req_reason_code() ==
          control::ReplanRequestReasonCode::REPLAN_REQ_STATION_REPLAN) {
    AINFO << "control_interactive_msg replan, all replan";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  } else {
    AINFO << "control_interactive_msg replan, speed replan";
    VehicleState vehicle_state_tmp = vehicle_state;
    vehicle_state_tmp.set_x(time_match_point.path_point().x());
    vehicle_state_tmp.set_y(time_match_point.path_point().y());
    vehicle_state_tmp.set_z(time_match_point.path_point().z());
    vehicle_state_tmp.set_heading(time_match_point.path_point().theta());
    vehicle_state_tmp.set_kappa(time_match_point.path_point().kappa());
    return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                            vehicle_state_tmp);
  }
}

}  // namespace planning
}  // namespace apollo
