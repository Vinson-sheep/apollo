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

#include "modules/planning/traffic_rules/crosswalk/crosswalk.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/planning/planning_base/proto/planning_status.pb.h"
#include "cyber/time/clock.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::hdmap::CrosswalkInfoConstPtr;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;
using apollo::perception::PerceptionObstacle;
using CrosswalkToStop =
    std::vector<std::pair<const hdmap::PathOverlap*, std::vector<std::string>>>;
using CrosswalkStopTimer =
    std::unordered_map<std::string, std::unordered_map<std::string, double>>;

bool Crosswalk::Init(const std::string& name,
                     const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<CrosswalkConfig>(&config_);
}

Status Crosswalk::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  // 在对遇到人行横道的情况进行处理时，首先从地图中获取所有的人行横道，然后其进行遍历
  if (!FindCrosswalks(reference_line_info)) {
    injector_->planning_context()->mutable_planning_status()->clear_crosswalk();
    return Status::OK();
  }
  // 做出决策
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void Crosswalk::MakeDecisions(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  // 获取人行横道状态
  auto* mutable_crosswalk_status = injector_->planning_context()
                                       ->mutable_planning_status()
                                       ->mutable_crosswalk();
  // 获取参考线决策
  auto* path_decision = reference_line_info->path_decision();
  // 获取当前参考线车辆对应的end_s值
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  // 需要停止的人行车道 （是一个vector）
  CrosswalkToStop crosswalks_to_stop;

  // read crosswalk_stop_timer from saved status
  // 人行横道停止计时器，开始停车时的时间戳
  CrosswalkStopTimer crosswalk_stop_timer;
  std::unordered_map<std::string, double> stop_times;
  for (const auto& stop_time : mutable_crosswalk_status->stop_time()) {
    stop_times.emplace(stop_time.obstacle_id(), stop_time.stop_timestamp_sec());
  }
  // 添加人行横道计时，障碍物id以及停止时间
  crosswalk_stop_timer.emplace(mutable_crosswalk_status->crosswalk_id(),
                               stop_times);
  // 获取完成的人行道
  const auto& finished_crosswalks =
      mutable_crosswalk_status->finished_crosswalk();
  // 获取参考线
  const auto& reference_line = reference_line_info->reference_line();
  for (auto crosswalk_overlap : crosswalk_overlaps_) {
    // 获取人行横道指针
    auto crosswalk_ptr = HDMapUtil::BaseMap().GetCrosswalkById(
        hdmap::MakeMapId(crosswalk_overlap->object_id));
    // 获取人行横道id
    std::string crosswalk_id = crosswalk_ptr->id().id();

    // 当车头驶过人行横道的距离超过阈值min_pass_s_distance时，如果当前的crosswalk_status中有crosswalk_id信息，
    // 且crosswalk_id就是当前正在遍历的crosswalk_id，则清除crosswalk_status中的ID及停止时间，即不需要停车；
    // skip crosswalk if master vehicle body already passes the stop line
    if (adc_front_edge_s - crosswalk_overlap->end_s >
        config_.min_pass_s_distance()) {
      if (mutable_crosswalk_status->has_crosswalk_id() &&
          mutable_crosswalk_status->crosswalk_id() == crosswalk_id) {
        mutable_crosswalk_status->clear_crosswalk_id();
        mutable_crosswalk_status->clear_stop_time();
      }
      // 跳过
      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id
             << "] crosswalk_overlap_end_s[" << crosswalk_overlap->end_s
             << "] adc_front_edge_s[" << adc_front_edge_s
             << "]. adc_front_edge passes crosswalk_end_s + buffer.";
      continue;
    }
    // 如果已经通过人行横道，则忽略
    // check if crosswalk already finished
    if (finished_crosswalks.end() != std::find(finished_crosswalks.begin(),
                                               finished_crosswalks.end(),
                                               crosswalk_id)) {
      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id << "] crosswalk_end_s["
             << crosswalk_overlap->end_s << "] finished already";
      continue;
    }
    // 遍历path_decision中的obstacle
    // 将所有符合条件的障碍物id添加道pedestrans，pedestrains添加over_lap
    std::vector<std::string> pedestrians;
    for (const auto* obstacle : path_decision->obstacles().Items()) {
      // 计算减速度，进入函数查看
      const double stop_deceleration = util::GetADCStopDeceleration(
          injector_->vehicle_state(), adc_front_edge_s,
          crosswalk_overlap->start_s);
      // 根据每个障碍物来检查是否需要停车
      bool stop = CheckStopForObstacle(reference_line_info, crosswalk_ptr,
                                       *obstacle, stop_deceleration);
      // 获取障碍物id
      const std::string& obstacle_id = obstacle->Id();
      // 获取感知障碍物类型
      const PerceptionObstacle& perception_obstacle = obstacle->Perception();
      PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
      // 获取障碍物类型名字
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);
      // 判断障碍物是否在当前参考线上
      // update stop timestamp on static pedestrian for watch timer
      const bool is_on_lane =
          reference_line.IsOnLane(obstacle->PerceptionSLBoundary());
      if (stop && !is_on_lane &&
          crosswalk_overlap->start_s - adc_front_edge_s <=
              config_.start_watch_timer_distance()) {
        // check on stop timer for static pedestrians/bicycles
        // if NOT on_lane ahead of adc
        const double kMaxStopSpeed = 0.3;
        // 获取障碍物速度
        auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                         perception_obstacle.velocity().y());
        // 如果障碍物速度小于等于0.3
        if (obstacle_speed <= kMaxStopSpeed) {
          if (crosswalk_stop_timer[crosswalk_id].count(obstacle_id) < 1) {
            // 如果障碍物不在crosswalk_stop_timer中，则插入
            // add timestamp
            ADEBUG << "add timestamp: obstacle_id[" << obstacle_id
                   << "] timestamp[" << Clock::NowInSeconds() << "]";
            crosswalk_stop_timer[crosswalk_id].insert(
                {obstacle_id, Clock::NowInSeconds()});
          } else {
            double stop_time = Clock::NowInSeconds() -
                               crosswalk_stop_timer[crosswalk_id][obstacle_id];
            ADEBUG << "stop_time: obstacle_id[" << obstacle_id << "] stop_time["
                   << stop_time << "]";
            if (stop_time >= config_.stop_timeout()) {
              stop = false;
            }
          }
        }
      }
      // 如果需要停车，则将obstacle_id放入pedestrains中
      if (stop) {
        pedestrians.push_back(obstacle_id);
        ADEBUG << "wait for: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      } else {
        ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "]";
      }
    } // 障碍物循环结束
    // pedestrains 非空，则将crosswalk_overlap 和 pedestrains放入crosswalks_to_stop
    if (!pedestrians.empty()) {
      crosswalks_to_stop.emplace_back(crosswalk_overlap, pedestrians);
      ADEBUG << "crosswalk_id[" << crosswalk_id << "] STOP";
    }
  } // over_lap循环结束
  // 作停止决策
  double min_s = std::numeric_limits<double>::max();
  // 第一个停止的人行车道
  hdmap::PathOverlap* firsts_crosswalk_to_stop = nullptr;
  for (auto crosswalk_to_stop : crosswalks_to_stop) {
    // build stop decision
    const auto* crosswalk_overlap = crosswalk_to_stop.first;
    ADEBUG << "BuildStopDecision: crosswalk[" << crosswalk_overlap->object_id
           << "] start_s[" << crosswalk_overlap->start_s << "]";
    std::string virtual_obstacle_id =
        CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;
    // 添加制定决策 障碍物id_over_lap起始s 停止距离 车辆停止原因 停止障碍物类型 决策标签 frame信息 参考线
    util::BuildStopDecision(
        virtual_obstacle_id, crosswalk_overlap->start_s,  
        config_.stop_distance(), // 1m
        StopReasonCode::STOP_REASON_CROSSWALK,
        crosswalk_to_stop.second, Getname(), frame, reference_line_info);
    // 根据crosswalk_to_stop的s取最小s来作为first_crosswalk_to_stop
    if (crosswalk_to_stop.first->start_s < min_s) {
      firsts_crosswalk_to_stop =
          const_cast<PathOverlap*>(crosswalk_to_stop.first);
      min_s = crosswalk_to_stop.first->start_s;
    }
  }
  // 更新crosswalk状态
  if (firsts_crosswalk_to_stop) {
    // update CrosswalkStatus
    // 获取人行横道ID
    std::string crosswalk = firsts_crosswalk_to_stop->object_id;
    mutable_crosswalk_status->set_crosswalk_id(crosswalk);
    mutable_crosswalk_status->clear_stop_time();
    for (const auto& timer : crosswalk_stop_timer[crosswalk]) {
      auto* stop_time = mutable_crosswalk_status->add_stop_time();
      stop_time->set_obstacle_id(timer.first);
      stop_time->set_stop_timestamp_sec(timer.second);
      ADEBUG << "UPDATE stop_time: id[" << crosswalk << "] obstacle_id["
             << timer.first << "] stop_timestamp[" << timer.second << "]";
    }

    // update CrosswalkStatus.finished_crosswalk
    mutable_crosswalk_status->clear_finished_crosswalk();
    for (auto crosswalk_overlap : crosswalk_overlaps_) {
      // 如果crosswalk_overlap小于第一个crosswalk_overlap则设置为结束的人行横道
      if (crosswalk_overlap->start_s < firsts_crosswalk_to_stop->start_s) {
        mutable_crosswalk_status->add_finished_crosswalk(
            crosswalk_overlap->object_id);
        ADEBUG << "UPDATE finished_crosswalk: " << crosswalk_overlap->object_id;
      }
    }
  }

  ADEBUG << "crosswalk_status: " << mutable_crosswalk_status->DebugString();
}

bool Crosswalk::FindCrosswalks(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  crosswalk_overlaps_.clear();
  const std::vector<hdmap::PathOverlap>& crosswalk_overlaps =
      reference_line_info->reference_line().map_path().crosswalk_overlaps();
  for (const hdmap::PathOverlap& crosswalk_overlap : crosswalk_overlaps) {
    crosswalk_overlaps_.push_back(&crosswalk_overlap);
  }
  return crosswalk_overlaps_.size() > 0;
}

bool Crosswalk::CheckStopForObstacle(
    ReferenceLineInfo* const reference_line_info,
    const CrosswalkInfoConstPtr crosswalk_ptr, const Obstacle& obstacle,
    const double stop_deceleration) {
  CHECK_NOTNULL(reference_line_info);
  // 检查人行横道ID
  std::string crosswalk_id = crosswalk_ptr->id().id();
  // 获取感知障碍物
  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  // 获取障碍物ID
  const std::string& obstacle_id = obstacle.Id();
  // 获取感知障碍物类型
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  // 获取感知障碍物类型名字
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);
  // 获取当前车辆start_s
  double adc_end_edge_s = reference_line_info->AdcSlBoundary().start_s();

  // 检查类型是否是pedestrain，bicycle，unknown movable or unknown。如果不是，此时不需要停车
  // check type
  if (obstacle_type != PerceptionObstacle::PEDESTRIAN &&
      obstacle_type != PerceptionObstacle::BICYCLE) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]. skip";
    return false;
  }

  // 判断障碍物是否在扩展后的人行道范围之内，不在则不停车
  // expand crosswalk polygon
  // note: crosswalk expanded area will include sideway area
  Vec2d point(perception_obstacle.position().x(),
              perception_obstacle.position().y());

  // 根据crosswalk的配置文件来扩展十字路口的Polygon
  const Polygon2d crosswalk_exp_poly =
      crosswalk_ptr->polygon().ExpandByDistance(config_.expand_s_distance());
  // 判断障碍物是否在十字路口内
  bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);
  // 如果不在十字路口内，则直接返回
  if (!in_expanded_crosswalk) {
    ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
           << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
           << "]: not in crosswalk expanded area";
    return false;
  }
  // 获取参考线
  const auto& reference_line = reference_line_info->reference_line();

  common::SLPoint obstacle_sl_point;
  // 获取障碍物点在参考线上的投影
  reference_line.XYToSL(perception_obstacle.position(), &obstacle_sl_point);
  // 获取障碍物sl_boundary
  auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  // 获取障碍物的l距离
  const double obstacle_l_distance =
      std::min(std::fabs(obstacle_sl_boundary.start_l()),
               std::fabs(obstacle_sl_boundary.end_l()));
  // 判断障碍物是否在道路上
  const bool is_on_lane =
      reference_line.IsOnLane(obstacle.PerceptionSLBoundary());
  // 判断障碍物的st边界是否为空，也就是是否穿过路径
  const bool is_on_road =
      reference_line.IsOnRoad(obstacle.PerceptionSLBoundary());
  // 判断障碍物的st边界是否为空，也就是是否穿过路径
  const bool is_path_cross = !obstacle.reference_line_st_boundary().IsEmpty();

  ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
         << "] crosswalk_id[" << crosswalk_id << "] obstacle_l["
         << obstacle_sl_point.l() << "] within_expanded_crosswalk_area["
         << in_expanded_crosswalk << "] obstacle_l_distance["
         << obstacle_l_distance << "] on_lane[" << is_on_lane << "] is_on_road["
         << is_on_road << "] is_path_cross[" << is_path_cross << "]";

  // 如果obstacle_l_distance大于等于8m，同时轨迹有重合，需要停车
  bool stop = false;
  if (obstacle_l_distance >= config_.stop_loose_l_distance()) {
    // (1) when obstacle_l_distance is big enough(>= loose_l_distance),
    //     STOP only if paths crosses
    // 障碍物轨迹穿过自身参考线路径，停车
    if (is_path_cross) {
      stop = true;
      ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle_id << "] type["
             << obstacle_type_name << "] crosswalk_id[" << crosswalk_id << "]";
    }
    // 如果obstacle_l_distance 小于等于6m，障碍物在当前道路前方，需要停车
  } else if (obstacle_l_distance <= config_.stop_strict_l_distance()) {
    if (is_on_road) {
      // (2) when l_distance <= strict_l_distance + on_road
      //     always STOP
      if (obstacle_sl_point.s() > adc_end_edge_s) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] s[" << obstacle_sl_point.s()
               << "] adc_end_edge_s[ " << adc_end_edge_s << "] crosswalk_id["
               << crosswalk_id << "] ON_ROAD";
      }
    } else {
      // 如果障碍物不在道路上，障碍物轨迹有重合，需要停车
      // (3) when l_distance <= strict_l_distance
      //     + NOT on_road(i.e. on crosswalk/median etc)
      //     STOP if paths cross
      if (is_path_cross) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "] PATH_CRSOSS";
      } else {
        // 如果obstacle_l_distance小于6m，障碍物轨迹没有重合但是在接近，需要停车
        // (4) when l_distance <= strict_l_distance
        //     + NOT on_road(i.e. on crosswalk/median etc)
        //     STOP if he pedestrian is moving toward the ego vehicle
        // 障碍物车速
        const auto obstacle_v = Vec2d(perception_obstacle.velocity().x(),
                                      perception_obstacle.velocity().y());
        // 获取当前车辆路径点
        const auto adc_path_point =
            Vec2d(injector_->ego_info()->start_point().path_point().x(),
                  injector_->ego_info()->start_point().path_point().y());
        // 获取障碍物位置点
        const auto ovstacle_position =
            Vec2d(perception_obstacle.position().x(),
                  perception_obstacle.position().y());
        // 获取障碍物点道当前车辆位置向量
        auto obs_to_adc = adc_path_point - ovstacle_position;
        const double kEpsilon = 1e-6;
        // 障碍物朝向自车移动
        if (obstacle_v.InnerProd(obs_to_adc) > kEpsilon) {
          stop = true;
          ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
                 << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
                 << "] MOVING_TOWARD_ADC";
        }
      }
    }
  } else {
    // (4) when l_distance is between loose_l and strict_l
    //     use history decision of this crosswalk to smooth unsteadiness

    // TODO(all): replace this temp implementation
    if (is_path_cross) {
      stop = true;
    }
    ADEBUG << "need_stop(between l1 & l2): obstacle_id[" << obstacle_id
           << "] type[" << obstacle_type_name << "] obstacle_l_distance["
           << obstacle_l_distance << "] crosswalk_id[" << crosswalk_id
           << "] USE_PREVIOUS_DECISION";
  }

  // 最后得到需要停车，但停车加速度大于config中在crosswalk场景下可以达到的最大减速度，同时，
  // obstacle L 大于 Strict L，则不需要停车 （减速度过大但忽略也足够安全）
  // check stop_deceleration
  if (stop) {
    if (stop_deceleration >= config_.max_stop_deceleration()) { // 6.0
      if (obstacle_l_distance > config_.stop_strict_l_distance()) { // 6.0
        // SKIP when stop_deceleration is too big but safe to ignore
        stop = false;
      }
      AWARN << "crosswalk_id[" << crosswalk_id << "] stop_deceleration["
            << stop_deceleration << "]";
    }
  }

  return stop;
}

}  // namespace planning
}  // namespace apollo
