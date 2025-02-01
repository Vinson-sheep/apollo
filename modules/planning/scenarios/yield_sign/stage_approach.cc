/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/yield_sign/stage_approach.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

StageResult YieldSignStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  // 读取scenario上下文
  auto scenario_context = GetContextAs<YieldSignContext>();

  // 将scenario配置保存在本地
  scenario_config_.CopyFrom(scenario_context->scenario_config);

  // 执行task
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "YieldSignStageApproach planning error";
  }

  // 提取第一条参考线
  const auto& reference_line_info = frame->reference_line_info().front();

  // 如果stage中已经没有任何让行标记，跳过stage
  if (scenario_context->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }

  // 遍历所有让行标记id
  for (const auto& yield_sign_overlap_id :
       scenario_context->current_yield_sign_overlap_ids) {
    // get overlap along reference line

    // 提取让行标记
    PathOverlap* current_yield_sign_overlap =
        reference_line_info.GetOverlapOnReferenceLine(
            yield_sign_overlap_id, ReferenceLineInfo::YIELD_SIGN);

    // 提取失败跳过
    if (!current_yield_sign_overlap) {
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_yield_sign_overlap->start_s, false);

    // 判断是否已经越过让行标记，如果是，跳过stage
    static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_pass_stop_sign =
        adc_front_edge_s - current_yield_sign_overlap->start_s;
    if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
      // passed stop line
      return FinishStage();
    }

    const double distance_adc_to_stop_line =
        current_yield_sign_overlap->start_s - adc_front_edge_s;
    // ADEBUG << "yield_sign_overlap_id[" << yield_sign_overlap_id << "] start_s["
    //        << current_yield_sign_overlap->start_s
    //        << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
    //        << "]";

    // 提取所有需要让行的障碍物
    bool yield_sign_done = false;
    if (distance_adc_to_stop_line <
        scenario_config_.max_valid_stop_distance()) {
      // close enough, check yield_sign clear
      yield_sign_done = true;
      const auto& path_decision = reference_line_info.path_decision();

      // 遍历所有决策过的障碍物
      for (const auto* obstacle : path_decision.obstacles().Items()) {

        // 提取id
        const std::string& obstacle_id = obstacle->Id();

        // 提取障碍物类型
        std::string obstacle_type_name =
            PerceptionObstacle_Type_Name(obstacle->Perception().type());
        // ADEBUG << "yield_sign[" << yield_sign_overlap_id << "] obstacle_id["
        //        << obstacle_id << "] type[" << obstacle_type_name << "]";

        // 忽略虚拟障碍物
        if (obstacle->IsVirtual()) {
          continue;
        }

        // 如果障碍物没有形状，忽略
        if (obstacle->reference_line_st_boundary().IsEmpty()) {
          continue;
        }

        // 如果障碍物t不在范围内，跳过
        static constexpr double kMinSTBoundaryT = 6.0;  // sec
        if (obstacle->reference_line_st_boundary().min_t() > kMinSTBoundaryT) {
          continue;
        }
        const double kepsilon = 1e-6;
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        // ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
        //        << obstacle->reference_line_st_boundary().min_t()
        //        << "] obstacle_st_min_s["
        //        << obstacle->reference_line_st_boundary().min_s()
        //        << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        // max st_min_t(sec) to ignore
        static constexpr double kIgnoreMaxSTMinT = 0.1;
        // min st_min_s(m) to ignore
        static constexpr double kIgnoreMinSTMinS = 15.0;

        // 如果障碍物在参考线内很少停留，跳过
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() < kIgnoreMaxSTMinT &&
            obstacle->reference_line_st_boundary().min_s() > kIgnoreMinSTMinS) {
          continue;
        }

        // 插入需要让行的障碍物
        injector_->planning_context()
            ->mutable_planning_status()
            ->mutable_yield_sign()
            ->add_wait_for_obstacle_id(obstacle->Id());

        yield_sign_done = false;
      }
    }

    // 如果没有障碍物需要让行，结束stage
    if (yield_sign_done) {
      return FinishStage();
    }
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult YieldSignStageApproach::FinishStage() {
  // update PlanningContext
  auto* yield_sign_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_yield_sign();
  yield_sign_status->mutable_done_yield_sign_overlap_id()->Clear();
  auto scenario_context = GetContextAs<YieldSignContext>();
  for (const auto& yield_sign_overlap_id :
       scenario_context->current_yield_sign_overlap_ids) {
    yield_sign_status->add_done_yield_sign_overlap_id(yield_sign_overlap_id);
  }
  yield_sign_status->clear_wait_for_obstacle_id();

  scenario_context->creep_start_time = Clock::NowInSeconds();

  next_stage_ = "YIELD_SIGN_CREEP";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
