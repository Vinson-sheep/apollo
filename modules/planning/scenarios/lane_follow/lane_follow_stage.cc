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
 **/

#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/constraint_checker/constraint_checker.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::util::PointFactory;
using apollo::cyber::Clock;

namespace {
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

// void LaneFollowStage::RecordObstacleDebugInfo(
//     ReferenceLineInfo* reference_line_info) {
//   if (!FLAGS_enable_record_debug) {
//     ADEBUG << "Skip record debug info";
//     return;
//   }
//   auto ptr_debug = reference_line_info->mutable_debug();

//   const auto path_decision = reference_line_info->path_decision();
//   for (const auto obstacle : path_decision->obstacles().Items()) {
//     auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
//     obstacle_debug->set_id(obstacle->Id());
//     obstacle_debug->mutable_sl_boundary()->CopyFrom(
//         obstacle->PerceptionSLBoundary());
//     const auto& decider_tags = obstacle->decider_tags();
//     const auto& decisions = obstacle->decisions();
//     if (decider_tags.size() != decisions.size()) {
//       AERROR << "decider_tags size: " << decider_tags.size()
//              << " different from decisions size:" << decisions.size();
//     }
//     for (size_t i = 0; i < decider_tags.size(); ++i) {
//       auto decision_tag = obstacle_debug->add_decision_tag();
//       decision_tag->set_decider_tag(decider_tags[i]);
//       decision_tag->mutable_decision()->CopyFrom(decisions[i]);
//     }
//   }
// }

// 对frame的多条参考线都生成对应轨迹
StageResult LaneFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  if (frame->reference_line_info().empty()) {
    return StageResult(StageStatusType::FINISHED);
  }

  bool has_drivable_reference_line = false;

  ADEBUG << "Number of reference lines:\t"
         << frame->mutable_reference_line_info()->size();

  unsigned int count = 0;
  StageResult result;

  // 遍历frame中的所有参考线信息
  // 一个info对应一条参考线，所有中间信息都保存在info中
  // 最终的轨迹也会保存在info中
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    // TODO(SHU): need refactor
    if (count++ == frame->mutable_reference_line_info()->size()) {
      break;
    }
    ADEBUG << "No: [" << count << "] Reference Line.";
    ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();

    // 如果已经找到参考线，则结束循环
    if (has_drivable_reference_line) {
      reference_line_info.SetDrivable(false);
      break;
    }

    // 对参考线执行所有的task，然后生成轨迹存储在info中
    result =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    // 对结果进行校验
    if (!result.HasError()) {
      // 如果非变道，则直接使用
      if (!reference_line_info.IsChangeLanePath()) {
        ADEBUG << "reference line is NOT lane change ref.";
        has_drivable_reference_line = true;
        continue;
      }
      // 如果代价合理，则直接使用
      if (reference_line_info.Cost() < kStraightForwardLineCost) {
        // If the path and speed optimization succeed on target lane while
        // under smart lane-change or IsClearToChangeLane under older version
        has_drivable_reference_line = true;
        reference_line_info.SetDrivable(true);
      } else {
        reference_line_info.SetDrivable(false);
        ADEBUG << "\tlane change failed";
      }
    } else {
      reference_line_info.SetDrivable(false);
    }
  }

  return has_drivable_reference_line
             ? result.SetStageStatus(StageStatusType::RUNNING)
             : result.SetStageStatus(StageStatusType::ERROR);
}

// 根据起始状态和帧信息，计算reference_line_info
StageResult LaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {

  // 如果没有变道，则需要添加跟随车道的代价
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  // ADEBUG << "planning start point:" << planning_start_point.DebugString();
  // ADEBUG << "Current reference_line_info is IsChangeLanePath: "
  //        << reference_line_info->IsChangeLanePath();

  StageResult ret;
  // 执行所有的task
  for (auto task : task_list_) {
    // const double start_timestamp = Clock::NowInSeconds();
    // const auto start_planning_perf_timestamp =
    //     std::chrono::duration<double>(
    //         std::chrono::system_clock::now().time_since_epoch())
    //         .count();

    ret.SetTaskStatus(task->Execute(frame, reference_line_info));

    // const double end_timestamp = Clock::NowInSeconds();
    // const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    // ADEBUG << "after task[" << task->Name()
    //        << "]:" << reference_line_info->PathSpeedDebugString();
    // ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
    // RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);

    // const auto end_planning_perf_timestamp =
    //     std::chrono::duration<double>(
    //         std::chrono::system_clock::now().time_since_epoch())
    //         .count();
    // const auto plnning_perf_ms =
    //     (end_planning_perf_timestamp - start_planning_perf_timestamp) * 1000;
    // AINFO << "Planning Perf: task name [" << task->Name() << "], "
    //       << plnning_perf_ms << " ms.";

    // 如果执行有误，报错
    if (ret.IsTaskError()) {
      // AERROR << "Failed to run tasks[" << task->Name()
      //        << "], Error message: " << ret.GetTaskStatus().error_message();
      break;
    }

    // TODO(SHU): disable reference line order changes for now
    // updated reference_line_info, because it is changed in
    // lane_change_decider by PrioritizeChangeLane().
    // reference_line_info = &frame->mutable_reference_line_info()->front();
    // ADEBUG << "Current reference_line_info is IsChangeLanePath: "
    //        << reference_line_info->IsChangeLanePath();
  }

  // RecordObstacleDebugInfo(reference_line_info);

  // check path and speed results for path or speed fallback
  // 如果task执行失败，使用默认task进一步处理
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  if (ret.IsTaskError()) {
    fallback_task_->Execute(frame, reference_line_info);
  }

  // 生成path和speed结合的轨迹，为什么不新增一个task去处理
  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    const std::string msg = "Fail to aggregate planning trajectory.";
    AERROR << msg;
    return ret.SetStageStatus(StageStatusType::ERROR, msg);
  }

  // 遍历所有的静态障碍物，计算目标停止点的s坐标
  // 此处为什么没有取最小值？
  // determine if there is a destination on reference line.
  double dest_stop_s = -1.0;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }

  // 遍历所有的障碍物，判断是否需要增加静态障碍物代价
  // 为什么不在优化层做这件事？？
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    // 跳过虚拟障碍物
    if (obstacle->IsVirtual()) {
      continue;
    }
    // 跳过动态障碍物
    if (!obstacle->IsStatic()) {
      continue;
    }
    // 只考虑已经停止的障碍物
    if (obstacle->LongitudinalDecision().has_stop()) {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s &&
            (dest_stop_s - reference_line_info->AdcSlBoundary().end_s()) <
                20.0) {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {
        static constexpr double kReferenceLineStaticObsCost = 1e3;
        reference_line_info->AddCost(kReferenceLineStaticObsCost);
      }
    }
  }

  // 对轨迹进行最终校验
  if (FLAGS_enable_trajectory_check) {
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {
      const std::string msg = "Current planning trajectory is not valid.";
      AERROR << msg;
      return ret.SetStageStatus(StageStatusType::ERROR, msg);
    }
  }

  // 将轨迹写入info，并执行
  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  ret.SetStageStatus(StageStatusType::RUNNING);
  return ret;
}

// 将xy停止点转换为sl坐标系
SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line) const {
  SLPoint sl_point;
  reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
  return sl_point;
}

}  // namespace planning
}  // namespace apollo
