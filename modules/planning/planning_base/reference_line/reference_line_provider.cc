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
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include "modules/planning/planning_base/reference_line/reference_line_provider.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/task/task.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleState;
using apollo::common::math::AngleDiff;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneWaypoint;
using apollo::hdmap::MapPathPoint;
using apollo::hdmap::RouteSegments;

ReferenceLineProvider::~ReferenceLineProvider() {}

ReferenceLineProvider::ReferenceLineProvider(
    const common::VehicleStateProvider *vehicle_state_provider,
    const ReferenceLineConfig *reference_line_config,
    const std::shared_ptr<relative_map::MapMsg> &relative_map)
    : vehicle_state_provider_(vehicle_state_provider) {
  current_pnc_map_ = nullptr;
  if (!FLAGS_use_navigation_mode) {
    relative_map_ = nullptr;
  } else {
    relative_map_ = relative_map;
  }

  ACHECK(cyber::common::GetProtoFromFile(FLAGS_smoother_config_filename,
                                         &smoother_config_))
      << "Failed to load smoother config file "
      << FLAGS_smoother_config_filename;

  // 根据配置初始化不同的参考线平滑器
  if (smoother_config_.has_qp_spline()) {
    smoother_.reset(new QpSplineReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_spiral()) {
    smoother_.reset(new SpiralReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_discrete_points()) {
    smoother_.reset(new DiscretePointsReferenceLineSmoother(smoother_config_));
  } else {
    ACHECK(false) << "unknown smoother config "
                  << smoother_config_.DebugString();
  }
  // Load pnc map plugins.
  pnc_map_list_.clear();
  // Set "apollo::planning::LaneFollowMap" as default if pnc_map_class is empty.
  // 加载地图，实际只有一张地图
  if (nullptr == reference_line_config ||
      reference_line_config->pnc_map_class().empty()) {
    const auto &pnc_map =
        apollo::cyber::plugin_manager::PluginManager::Instance()
            ->CreateInstance<planning::PncMapBase>(
                "apollo::planning::LaneFollowMap");
    pnc_map_list_.emplace_back(pnc_map);
  } else {
    const auto &pnc_map_names = reference_line_config->pnc_map_class();
    for (const auto &map_name : pnc_map_names) {
      const auto &pnc_map =
          apollo::cyber::plugin_manager::PluginManager::Instance()
              ->CreateInstance<planning::PncMapBase>(map_name);
      pnc_map_list_.emplace_back(pnc_map);
    }
  }

  is_initialized_ = true;
}

// 命令居然是地图来执行的？
bool ReferenceLineProvider::UpdatePlanningCommand(
    const planning::PlanningCommand &command) {
  std::lock_guard<std::mutex> routing_lock(routing_mutex_);
  bool find_matched_pnc_map = false;
  // 寻找能执行指令的地图为当前地图
  for (const auto &pnc_map : pnc_map_list_) {
    if (pnc_map->CanProcess(command)) {
      current_pnc_map_ = pnc_map;
      find_matched_pnc_map = true;
      break;
    }
  }
  // 如果找不到，报异常
  if (nullptr == current_pnc_map_) {
    AERROR << "Cannot find pnc map to process input command!"
           << command.DebugString();
    return false;
  if (!find_matched_pnc_map) {
    AWARN << "Find no pnc map for the input command and the old one will be "
             "used!";
  }
  // 让pnc_map执行命令
  // Update routing in pnc_map
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  if (current_pnc_map_->IsNewPlanningCommand(command)) {
    is_new_command_ = true;
    // command中包含多条routing路径
    if (!current_pnc_map_->UpdatePlanningCommand(command)) {
      AERROR << "Failed to update routing in pnc map: "
             << command.DebugString();
      return false;
    }
  }
  planning_command_ = command;
  has_planning_command_ = true;
  return true;
}

std::vector<routing::LaneWaypoint>
ReferenceLineProvider::FutureRouteWaypoints() {
  if (!FLAGS_use_navigation_mode && nullptr != current_pnc_map_) {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    return current_pnc_map_->FutureRouteWaypoints();
  }

  // return an empty routing::LaneWaypoint vector in Navigation mode.
  return std::vector<routing::LaneWaypoint>();
}

void ReferenceLineProvider::GetEndLaneWayPoint(
    std::shared_ptr<routing::LaneWaypoint> &end_point) const {
  if (nullptr == current_pnc_map_) {
    end_point = nullptr;
    return;
  }
  current_pnc_map_->GetEndLaneWayPoint(end_point);
}

hdmap::LaneInfoConstPtr ReferenceLineProvider::GetLaneById(
    const hdmap::Id &id) const {
  if (nullptr == current_pnc_map_) {
    return nullptr;
  }
  return current_pnc_map_->GetLaneById(id);
}

void ReferenceLineProvider::UpdateVehicleState(
    const VehicleState &vehicle_state) {
  std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
  vehicle_state_ = vehicle_state;
}

// 参考线生成器开始函数
bool ReferenceLineProvider::Start() {
  if (FLAGS_use_navigation_mode) {
    return true;
  }
  // 
  if (!is_initialized_) {
    AERROR << "ReferenceLineProvider has NOT been initiated.";
    return false;
  }
  // 进入参考线生成线程
  if (FLAGS_enable_reference_line_provider_thread) {
    task_future_ = cyber::Async(&ReferenceLineProvider::GenerateThread, this);
  }
  return true;
}

void ReferenceLineProvider::Stop() {
  is_stop_ = true;
  if (FLAGS_enable_reference_line_provider_thread) {
    task_future_.get();
  }
}

void ReferenceLineProvider::Reset() {
  std::lock_guard<std::mutex> lock(routing_mutex_);
  has_planning_command_ = false;
  is_new_command_ = false;
  reference_lines_.clear();
  route_segments_.clear();
  is_reference_line_updated_ = false;
  planning_command_.Clear();
  while (!reference_line_history_.empty()) {
    reference_line_history_.pop();
  }
}

// 直接更新不行吗，没看出来有时间上的节约
void ReferenceLineProvider::UpdateReferenceLine(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &route_segments) {
  if (reference_lines.size() != route_segments.size()) {
    AERROR << "The calculated reference line size(" << reference_lines.size()
           << ") and route_segments size(" << route_segments.size()
           << ") are different";
    return;
  }
  if (reference_lines.empty()) {
    return;
  }
  // 如果参考线数目不一样，则参考线不一样
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);
  if (reference_lines_.size() != reference_lines.size()) {
    reference_lines_ = reference_lines;
    route_segments_ = route_segments;
  } 
  // 否则，循环查看参考线内容
  else {
    auto segment_iter = route_segments.begin();
    auto internal_iter = reference_lines_.begin();
    auto internal_segment_iter = route_segments_.begin();
    for (auto iter = reference_lines.begin();
         iter != reference_lines.end() &&
         segment_iter != route_segments.end() &&
         internal_iter != reference_lines_.end() &&
         internal_segment_iter != route_segments_.end();
         ++iter, ++segment_iter, ++internal_iter, ++internal_segment_iter) {
      if (iter->reference_points().empty()) {
        *internal_iter = *iter;
        *internal_segment_iter = *segment_iter;
        continue;
      }
      if (common::util::SamePointXY(
              iter->reference_points().front(),
              internal_iter->reference_points().front()) &&
          common::util::SamePointXY(iter->reference_points().back(),
                                    internal_iter->reference_points().back()) &&
          std::fabs(iter->Length() - internal_iter->Length()) <
              common::math::kMathEpsilon) {
        continue;
      }
      *internal_iter = *iter;
      *internal_segment_iter = *segment_iter;
    }
  }
  // update history
  reference_line_history_.push(reference_lines_);
  route_segments_history_.push(route_segments_);
  static constexpr int kMaxHistoryNum = 3;
  if (reference_line_history_.size() > kMaxHistoryNum) {
    reference_line_history_.pop();
    route_segments_history_.pop();
  }
}

// 还要特地创造线程？
void ReferenceLineProvider::GenerateThread() {
  // is_stop_是一个atomic参数在ReferenceLineProvider::Stop()设置为true
  while (!is_stop_) {
    static constexpr int32_t kSleepTime = 50;  // milliseconds
    cyber::SleepFor(std::chrono::milliseconds(kSleepTime));
    // 计算起始时间
    const double start_time = Clock::NowInSeconds();
    // 
    if (!has_planning_command_) {
      continue;
    }
    // 路径点信息集
    std::list<ReferenceLine> reference_lines;
    // LaneInfo信息集
    std::list<hdmap::RouteSegments> segments;
    if (!CreateReferenceLine(&reference_lines, &segments)) {
      // 创建参考线失败 is_reference_line_updated_ 为atomic参数
      is_reference_line_updated_ = false;
      AERROR << "Fail to get reference line";
      continue;
    }
    // 更新参考线
    UpdateReferenceLine(reference_lines, segments);
    // 计算终止时间
    const double end_time = Clock::NowInSeconds();
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    // 计算参考线生成程序耗时
    last_calculation_time_ = end_time - start_time;
    // 参考线更新成功 std::atomic<bool>
    is_reference_line_updated_ = true;
  }
}

// 获取上次参考线更新的时间
double ReferenceLineProvider::LastTimeDelay() {
  if (FLAGS_enable_reference_line_provider_thread &&
      !FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    return last_calculation_time_;
  } else {
    return last_calculation_time_;
  }
}

// 最核心的函数
// 根据relative_map或者pnc_map计算参考线
bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);
  if (!has_planning_command_) {
    return true;
  }

  // 从relative_map地图中获取参考线和车道段，并更新其内容
  // 这个过程没有涉及任何平滑？
  if (FLAGS_use_navigation_mode) {
    double start_time = Clock::NowInSeconds();
    bool result = GetReferenceLinesFromRelativeMap(reference_lines, segments);
    if (!result) {
      AERROR << "Failed to get reference line from relative map";
    }
    double end_time = Clock::NowInSeconds();
    last_calculation_time_ = end_time - start_time;
    return result;
  }

  // 如果已经存在参考线，直接使用
  if (FLAGS_enable_reference_line_provider_thread) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    if (!reference_lines_.empty()) {
      reference_lines->assign(reference_lines_.begin(), reference_lines_.end());
      segments->assign(route_segments_.begin(), route_segments_.end());
      return true;
    }
  } 
  // 否则，基于Pnc地图生成一条参考线
  // 这个过程涉及平滑
  else {
    double start_time = Clock::NowInSeconds();
    if (CreateReferenceLine(reference_lines, segments)) {
      UpdateReferenceLine(*reference_lines, *segments);
      double end_time = Clock::NowInSeconds();
      last_calculation_time_ = end_time - start_time;
      return true;
    }
  }

  AINFO << "Reference line is NOT ready.";
  if (reference_line_history_.empty()) {
    AINFO << "Failed to use reference line latest history";
    return false;
  }

  // 如果生成失败，使用历史参考线
  reference_lines->assign(reference_line_history_.back().begin(),
                          reference_line_history_.back().end());
  segments->assign(route_segments_history_.back().begin(),
                   route_segments_history_.back().end());
  AWARN << "Use reference line from history!";
  return true;
}

// 首先获取第一条参考线的迭代器，然后遍历所有的参考线
// 如果当前的参考线为允许变道参考线，则将第一条参考线更换为当前迭代器所指向的参考线
// 注意，可变车道为按迭代器的顺序求取，一旦发现可变车道，即退出循环
void ReferenceLineProvider::PrioritizeChangeLane(
    std::list<hdmap::RouteSegments> *route_segments) {
  CHECK_NOTNULL(route_segments);
  auto iter = route_segments->begin();
  while (iter != route_segments->end()) {
    if (!iter->IsOnSegment()) { // 如果当前的参考线为允许变道参考线
      // 则将第一条参考线更换为当前迭代器所指的参考线
      route_segments->splice(route_segments->begin(), *route_segments, iter);
      break;
    }
    ++iter;
  }
}

// 基于高清地图和当前车道信息计算参考线
// 这里的relative_map只是一个指针，似乎是个数据体而已，不是具体的地图
bool ReferenceLineProvider::GetReferenceLinesFromRelativeMap(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_GE(relative_map_->navigation_path_size(), 0);
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  if (relative_map_->navigation_path().empty()) {
    AERROR << "There isn't any navigation path in current relative map.";
    return false;
  }

  auto *hdmap = HDMapUtil::BaseMapPtr(*relative_map_);
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  // 获取车辆行驶中的的车道线
  // 1.get adc current lane info ,such as lane_id,lane_priority,neighbor lanes
  std::unordered_set<std::string> navigation_lane_ids;
  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto lane_id = path_pair.first;
    navigation_lane_ids.insert(lane_id);
  }
  // 如果没有车道线，直接返回
  if (navigation_lane_ids.empty()) {
    AERROR << "navigation path ids is empty";
    return false;
  }
  // 否则，选取最近的车道，计算投影点
  // get current adc lane info by vehicle state
  common::VehicleState vehicle_state = vehicle_state_provider_->vehicle_state();
  hdmap::LaneWaypoint adc_lane_way_point;
  if (!GetNearestWayPointFromNavigationPath(vehicle_state, navigation_lane_ids,
                                            &adc_lane_way_point)) {
    return false;
  }

  // 获取车道优先级，用于打印
  const std::string adc_lane_id = adc_lane_way_point.lane->id().id();
  auto *adc_navigation_path = apollo::common::util::FindOrNull(
      relative_map_->navigation_path(), adc_lane_id);
  if (adc_navigation_path == nullptr) {
    AERROR << "adc lane cannot be found in relative_map_->navigation_path";
    return false;
  }
  const uint32_t adc_lane_priority = adc_navigation_path->path_priority();

  // 获取所有左车道
  // get adc left neighbor lanes
  std::vector<std::string> left_neighbor_lane_ids;
  auto left_lane_ptr = adc_lane_way_point.lane;
  while (left_lane_ptr != nullptr &&
         left_lane_ptr->lane().left_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        left_lane_ptr->lane().left_neighbor_forward_lane_id(0);
    left_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    left_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " left neighbor size : " << left_neighbor_lane_ids.size();
  for (const auto &neighbor : left_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " left neighbor : " << neighbor;
  }

  // 获取所有右车道
  // get adc right neighbor lanes
  std::vector<std::string> right_neighbor_lane_ids;
  auto right_lane_ptr = adc_lane_way_point.lane;
  while (right_lane_ptr != nullptr &&
         right_lane_ptr->lane().right_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        right_lane_ptr->lane().right_neighbor_forward_lane_id(0);
    right_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    right_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " right neighbor size : " << right_neighbor_lane_ids.size();
  for (const auto &neighbor : right_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " right neighbor : " << neighbor;
  }

  //
  // 2.get the higher priority lane info list which priority higher
  // than current lane and get the highest one as the target lane
  using LaneIdPair = std::pair<std::string, uint32_t>;
  std::vector<LaneIdPair> high_priority_lane_pairs;
  ADEBUG << "relative_map_->navigation_path_size = "
         << relative_map_->navigation_path_size();
  // 遍历所有候选车道
  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto lane_id = path_pair.first;
    const uint32_t priority = path_pair.second.path_priority();
    ADEBUG << "lane_id = " << lane_id << " priority = " << priority
           << " adc_lane_id = " << adc_lane_id
           << " adc_lane_priority = " << adc_lane_priority;
    // the smaller the number, the higher the priority
    // 如果候选车道比当前车道优先级更高，则插入候选车道
    if (adc_lane_id != lane_id && priority < adc_lane_priority) {
      high_priority_lane_pairs.emplace_back(lane_id, priority);
    }
  }

  // 排序，获取最优先车道。如果不是当前车道，则需要换道
  // get the target lane
  bool is_lane_change_needed = false;
  LaneIdPair target_lane_pair;
  if (!high_priority_lane_pairs.empty()) {
    std::sort(high_priority_lane_pairs.begin(), high_priority_lane_pairs.end(),
              [](const LaneIdPair &left, const LaneIdPair &right) {
                return left.second < right.second;
              });
    ADEBUG << "need to change lane";
    // the highest priority lane as the target navigation lane
    target_lane_pair = high_priority_lane_pairs.front();
    is_lane_change_needed = true;
  }

  // 判断是左车道还是右车道换道，并且获得目标邻近车道
  // 3.get current lane's the nearest neighbor lane to the target lane
  // and make sure it position is left or right on the current lane
  routing::ChangeLaneType lane_change_type = routing::FORWARD;
  std::string nearest_neighbor_lane_id;
  if (is_lane_change_needed) {
    // target on the left of adc
    if (left_neighbor_lane_ids.end() !=
        std::find(left_neighbor_lane_ids.begin(), left_neighbor_lane_ids.end(),
                  target_lane_pair.first)) {
      // take the id of the first adjacent lane on the left of adc as
      // the nearest_neighbor_lane_id
      lane_change_type = routing::LEFT;
      nearest_neighbor_lane_id =
          adc_lane_way_point.lane->lane().left_neighbor_forward_lane_id(0).id();
    } else if (right_neighbor_lane_ids.end() !=
               std::find(right_neighbor_lane_ids.begin(),
                         right_neighbor_lane_ids.end(),
                         target_lane_pair.first)) {
      // target lane on the right of adc
      // take the id  of the first adjacent lane on the right of adc as
      // the nearest_neighbor_lane_id
      lane_change_type = routing::RIGHT;
      nearest_neighbor_lane_id = adc_lane_way_point.lane->lane()
                                     .right_neighbor_forward_lane_id(0)
                                     .id();
    }
  }

  // 遍历附近的车道，提取车道段信息和车道离散点
  for (const auto &path_pair : relative_map_->navigation_path()) {
    const auto &lane_id = path_pair.first;
    const auto &path_points = path_pair.second.path().path_point();
    auto lane_ptr = hdmap->GetLaneById(hdmap::MakeMapId(lane_id));
    RouteSegments segment;
    segment.emplace_back(lane_ptr, 0.0, lane_ptr->total_length());
    segment.SetCanExit(true);
    segment.SetId(lane_id);
    segment.SetNextAction(routing::FORWARD);
    segment.SetStopForDestination(false);
    segment.SetPreviousAction(routing::FORWARD);

    if (is_lane_change_needed) {
      if (lane_id == nearest_neighbor_lane_id) {
        ADEBUG << "adc lane_id = " << adc_lane_id
               << " nearest_neighbor_lane_id = " << lane_id;
        segment.SetIsNeighborSegment(true);
        segment.SetPreviousAction(lane_change_type);
      } else if (lane_id == adc_lane_id) {
        segment.SetIsOnSegment(true); // 位于当前车道
        segment.SetNextAction(lane_change_type);
      }
    }

    segments->emplace_back(segment);
    std::vector<ReferencePoint> ref_points;
    for (const auto &path_point : path_points) {
      // 插入的是SL坐标
      ref_points.emplace_back(
          MapPathPoint{Vec2d{path_point.x(), path_point.y()},
                       path_point.theta(),
                       LaneWaypoint(lane_ptr, path_point.s())},
          path_point.kappa(), path_point.dkappa());
    }
    reference_lines->emplace_back(ref_points.begin(), ref_points.end());
    reference_lines->back().SetPriority(path_pair.second.path_priority());
  }
  return !segments->empty();
}

// 
bool ReferenceLineProvider::GetNearestWayPointFromNavigationPath(
    const common::VehicleState &state,
    const std::unordered_set<std::string> &navigation_lane_ids,
    hdmap::LaneWaypoint *waypoint) {
  const double kMaxDistance = 10.0;
  waypoint->lane = nullptr;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  auto point = common::util::PointFactory::ToPointENU(state);
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "vehicle state is invalid";
    return false;
  }
  auto *hdmap = HDMapUtil::BaseMapPtr();
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  // 根据朝向和位置过滤目标车道线
  // get all adc direction lanes from map in kMaxDistance range
  // by vehicle point in map
  const int status = hdmap->GetLanesWithHeading(
      point, kMaxDistance, state.heading(), M_PI / 2.0, &lanes);
  if (status < 0) {
    AERROR << "failed to get lane from point " << point.ShortDebugString();
    return false;
  }

  // 筛选出合理的车道信息
  // get lanes that exist in both map and navigation paths as valid lanes
  std::vector<hdmap::LaneInfoConstPtr> valid_lanes;
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](hdmap::LaneInfoConstPtr ptr) {
                 return navigation_lane_ids.count(ptr->lane().id().id()) > 0;
               });
  
  // 如果为空，则返回
  if (valid_lanes.empty()) {
    AERROR << "no valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }

  // 遍历所有候选车道
  // get nearest lane waypoints for current adc position
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &lane : valid_lanes) {

    // 将点进行投影，判断是否落在车道上
    // project adc point to lane to check if it is out of lane range
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, state.heading(), &s, &l)) {
      continue;
    }
    static constexpr double kEpsilon = 1e-6;
    if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
      continue;
    }

    // 计算投影点和距离
    // get the nearest distance between adc point and lane
    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    // record the near distance lane

    // 获取最近的投影点
    if (distance < min_distance) {
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "failed to get projection for map_point "
               << map_point.DebugString();
        continue;
      }
      min_distance = distance;
      waypoint->lane = lane;
      waypoint->s = s;
    }
  }

  if (waypoint->lane == nullptr) {
    AERROR << "failed to find nearest point " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

// 基于pnc地图获取segments
bool ReferenceLineProvider::CreateRouteSegments(
    const common::VehicleState &vehicle_state,
    std::list<hdmap::RouteSegments> *segments) {
  // 调用pnc地图进行路由，获取segmentsd
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (!current_pnc_map_->GetRouteSegments(vehicle_state, segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }
  for (auto &seg : *segments) {
    ADEBUG << seg.DebugString();
  }
  // 如果优先换道，则插入非当前车道的segments ？？
  if (FLAGS_prioritize_change_lane) {
    PrioritizeChangeLane(segments);
  }
  return !segments->empty();
}

// reference_lines和segments其实是参考线不同的表征
bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);
  // 获取自车当前车辆状态，也就是定位位置
  common::VehicleState vehicle_state;
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state = vehicle_state_;
  }
  // 获取routing内容
  planning::PlanningCommand command;
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    command = planning_command_;
  }
  if (nullptr == current_pnc_map_) {
    AERROR << "Current pnc map is null! " << command.DebugString();
    return false;
  }
  // 基于pnc地图路由获取segments
  if (!CreateRouteSegments(vehicle_state, segments)) {
    AERROR << "Failed to create reference line from routing";
    return false;
  }
  // 如果有新routing，或者不使用参考线拼接
  if (is_new_command_ || !FLAGS_enable_reference_line_stitching) {
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      // 如果优化失败，则剔除
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } 
      else {
        // 参考线优化成功
        // 计算sl坐标。如果失败，说明平滑路径有问题，报错
        common::SLPoint sl;
        if (!reference_lines->back().XYToSL(
                vehicle_state.heading(),
                common::math::Vec2d(vehicle_state.x(), vehicle_state.y()),
                &sl)) {
          AWARN << "Failed to project point: {" << vehicle_state.x() << ","
                << vehicle_state.y() << "} to stitched reference line";
        }
        // 收缩参考线和segment
        Shrink(sl, &reference_lines->back(), &(*iter));
        ++iter;
      }
    }
    is_new_command_ = false;
    return true;
  } else {  // stitching reference line 参考线拼接
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!ExtendReferenceLine(vehicle_state, &(*iter),
                               &reference_lines->back())) {
        AERROR << "Failed to extend reference line";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
  }
  return true;
}

// 参考线连接 扩展参考线
// 其主要功能在于每一帧的规划大多数情况下参考线的很大一部分区域是可以复用的，这样只需要对
// 这帧延长的部分加入进上一帧的参考线即可。拼接的好处在于一是减少许多的重复计算，参考线
// 的平滑的优化变量越多就越耗时，只对延长部分进行平滑操作显然节省了大量的计算时间；另一
// 方面在于保证参考线的稳定性，作为后续优化算法的重要基础，参考线前后帧的稳定性非常重要，
// 拼接可以减少后续轨迹层的跳变。
bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState &state,
                                                RouteSegments *segments,
                                                ReferenceLine *reference_line) {
  // 遍历以往的参考线，寻找连接当前参考线段的段
  // 设置属性
  RouteSegments segment_properties;
  segment_properties.SetProperties(*segments);
  // 提取上一帧std::list<hdmap::RouteSegment>的begin数据
  auto prev_segment = route_segments_.begin();
  // 提取上一帧std::list<ReferenceLine>的begin数据
  auto prev_ref = reference_lines_.begin();
  while (prev_segment != route_segments_.end()) {
    // 判断上一帧的数据哪个passage与当前计算出来的RouteSegments是否是连接的segments，
    // 互相检查起始点和终点是否在彼此的路径
    // 如果在，则直接退出循环，说明两条passage是匹配的，那么将其记录下来
    if (prev_segment->IsConnectedSegment(*segments)) {
      break;
    }
    ++prev_segment;
    ++prev_ref;
  }
  // 如果检查到最后上一周期的参考线与新的都不连接，则直接优化更新
  if (prev_segment == route_segments_.end()) {
    if (!route_segments_.empty() && segments->IsOnSegment()) {
      AWARN << "Current route segment is not connected with previous route "
               "segment";
    }
    return SmoothRouteSegment(*segments, reference_line);
  }
  // 用当前车辆位置在上一帧route_segments中计算投影点，若计算失败则平滑后返回
  common::SLPoint sl_point;
  Vec2d vec2d(state.x(), state.y());
  LaneWaypoint waypoint;
  if (!prev_segment->GetProjection(vec2d, state.heading(), &sl_point,
                                   &waypoint)) {
    AWARN << "Vehicle current point: " << vec2d.DebugString()
          << " not on previous reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  // 计算RouteSegments长度
  const double prev_segment_length = RouteSegments::Length(*prev_segment);
  // 计算在上一帧参考线的剩余距离
  const double remain_s = prev_segment_length - sl_point.s();
  // 根据当前车速计算预瞄距离
  const double look_forward_required_distance =
      planning::PncMapBase::LookForwardDistance(state.linear_velocity());
  // 如果剩余距离大于预瞄距离，则不需要优化，直接将上一帧率数据赋值给当前数据
  if (remain_s > look_forward_required_distance) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Reference line remain " << remain_s
           << ", which is more than required " << look_forward_required_distance
           << " and no need to extend";
    return true;
  }
  // 剩余距离小于预瞄距离
  // future_start_s，上一帧车辆位置的s与(上一帧参考长度 - 20) 取大
  // 如果sl_point.s()大，说明车辆已经驶出上一帧车道
  double future_start_s =
      std::max(sl_point.s(), prev_segment_length -
                                 FLAGS_reference_line_stitch_overlap_distance);
  // future_end_s，上一帧参考长度 + 50
  double future_end_s =
      prev_segment_length + FLAGS_look_forward_extend_distance;
  RouteSegments shifted_segments;
  std::unique_lock<std::mutex> lock(pnc_map_mutex_);
  // 调用pnc_map下的ExtendSegments函数，对route_segments查找前继及后继车道，从而实现道路段拓展
  if (!current_pnc_map_->ExtendSegments(*prev_segment, future_start_s,
                                        future_end_s, &shifted_segments)) {
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(*segments, reference_line);
  }
  lock.unlock();
  // 如果扩展扩展后的route_segments的最后一个点还在上一帧的route_segments上，则直接将上一帧的
  // route_segments返回
  if (prev_segment->IsWaypointOnSegment(shifted_segments.LastWaypoint())) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Could not further extend reference line";
    return true;
  }
  hdmap::Path path(shifted_segments);
  ReferenceLine new_ref(path);
  // 平滑前置被强制锁定的referenceline，即上一帧重复的点无需再重复平滑了
  if (!SmoothPrefixedReferenceLine(*prev_ref, new_ref, reference_line)) {
    AWARN << "Failed to smooth forward shifted reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  // 拼接参考线
  if (!reference_line->Stitch(*prev_ref)) {
    AWARN << "Failed to stitch reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!shifted_segments.Stitch(*prev_segment)) {
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(*segments, reference_line);
  }
  *segments = shifted_segments;
  segments->SetProperties(segment_properties);
  common::SLPoint sl;
  if (!reference_line->XYToSL(state.heading(), vec2d, &sl)) {
    AWARN << "Failed to project point: " << vec2d.DebugString()
          << " to stitched reference line";
  }
  // 收缩参考线，主要针对U型弯等曲率过大的弯道，收缩至角度与当前路点航向角的差在5/6 PI之内
  return Shrink(sl, reference_line, segments);
}

// 收缩参考线，主要针对U型弯等曲率过大的弯道，收缩至角度与当前路点航向角的差在5/6 PI之内
bool ReferenceLineProvider::Shrink(const common::SLPoint &sl,
                                   ReferenceLine *reference_line,
                                   RouteSegments *segments) {
  // shrink reference line
  double new_backward_distance = sl.s();
  double new_forward_distance = reference_line->Length() - sl.s();
  bool need_shrink = false;
  // 收缩参考线
  if (sl.s() > planning::FLAGS_look_backward_distance * 1.5) {
    ADEBUG << "reference line back side is " << sl.s()
           << ", shrink reference line: origin length: "
           << reference_line->Length();
    new_backward_distance = planning::FLAGS_look_backward_distance;
    need_shrink = true;
  }
  // 估算前向裁剪距离
  // check heading
  const auto index = reference_line->GetNearestReferenceIndex(sl.s());
  const auto &ref_points = reference_line->reference_points();
  const double cur_heading = ref_points[index].heading();
  auto last_index = index;
  while (last_index < ref_points.size() &&
         std::fabs(AngleDiff(cur_heading, ref_points[last_index].heading())) <
             FLAGS_reference_line_max_forward_heading_diff) {
    ++last_index;
  }
  --last_index;
  if (last_index != ref_points.size() - 1) {
    need_shrink = true;
    common::SLPoint forward_sl;
    reference_line->XYToSL(ref_points[last_index], &forward_sl);
    new_forward_distance = forward_sl.s() - sl.s();
  }

  // 再次估算后向距离
  // check backward heading
  last_index = index;
  while (last_index > 0 &&
         abs(AngleDiff(cur_heading, ref_points[last_index].heading())) <
             FLAGS_reference_line_max_backward_heading_diff) {
    --last_index;
  }
  if (last_index != 0) {
    need_shrink = true;
    common::SLPoint backward_sl;
    reference_line->XYToSL(ref_points[last_index], &backward_sl);
    new_backward_distance = sl.s() - backward_sl.s();
  }

  // 进行裁剪
  if (need_shrink) {
    if (!reference_line->Segment(sl.s(), new_backward_distance,
                                 new_forward_distance)) {
      AWARN << "Failed to shrink reference line";
    }
    if (!segments->Shrink(sl.s(), new_backward_distance,
                          new_forward_distance)) {
      AWARN << "Failed to shrink route segment";
    }
  }
  return true;
}

// 校验平滑后的参考线
bool ReferenceLineProvider::IsReferenceLineSmoothValid(
    const ReferenceLine &raw, const ReferenceLine &smoothed) const {
  static constexpr double kReferenceLineDiffCheckStep = 10.0;
  for (double s = 0.0; s < smoothed.Length();
       s += kReferenceLineDiffCheckStep) {
    auto xy_new = smoothed.GetReferencePoint(s);

    // 如果旧路径无法映射新路径，说明新路径不光滑
    common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, &sl_new)) {
      AERROR << "Fail to change xy point on smoothed reference line to sl "
                "point respect to raw reference line.";
      return false;
    }
    // 如果新路径偏离旧路径，则优化过度
    const double diff = std::fabs(sl_new.l());
    if (diff > FLAGS_smoothed_reference_line_max_diff) {
      AERROR << "Fail to provide reference line because too large diff "
                "between smoothed and raw reference lines. diff: "
             << diff;
      return false;
    }
  }
  return true;
}
// 根据道路情况生成路径锚点
AnchorPoint ReferenceLineProvider::GetAnchorPoint(
    const ReferenceLine &reference_line, double s) const {
  AnchorPoint anchor;
  // 获取纵向边界
  anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
  auto ref_point = reference_line.GetReferencePoint(s);

  // 如果lane_waypoints为空
  if (ref_point.lane_waypoints().empty()) {
    anchor.path_point = ref_point.ToPathPoint(s);
    // 根据配置文件获取横向边界
    anchor.lateral_bound = smoother_config_.max_lateral_boundary_bound();
    return anchor;
  }

  // 根据配置文件获取车辆宽度
  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  // 创建单位向量，垂直点航向角度
  const Vec2d left_vec =
      Vec2d::CreateUnitVec2d(ref_point.heading() + M_PI / 2.0);
  // 根据waypoint获取车道左侧和右侧边界宽度
  auto waypoint = ref_point.lane_waypoints().front();
  double left_width = 0.0;
  double right_width = 0.0;
  waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width);
  const double kEpislon = 1e-8;
  double effective_width = 0.0;

  // shrink width by vehicle width, curb
  // 根据车辆宽度收缩边界宽度
  double safe_lane_width = left_width + right_width;
  // 减去车辆宽度
  safe_lane_width -= adc_width;
  bool is_lane_width_safe = true; // 车道宽度是否安全

  if (safe_lane_width < kEpislon) {
    ADEBUG << "lane width [" << left_width + right_width << "] "
           << "is smaller than adc width [" << adc_width << "]";
    effective_width = kEpislon;
    is_lane_width_safe = false;
  }

  // 中心偏移
  double center_shift = 0.0;
  // 如果是右侧边界马路牙子
  if (hdmap::RightBoundaryType(waypoint) == hdmap::LaneBoundaryType::CURB) {
    // 如果有马路牙子，则减去相应阈值
    safe_lane_width -= smoother_config_.curb_shift();
    if (safe_lane_width < kEpislon) {
      ADEBUG << "lane width smaller than adc width and right curb shift";
      effective_width = kEpislon;
      is_lane_width_safe = false;
    } else {
      // 中心偏移0.1
      center_shift += 0.5 * smoother_config_.curb_shift();
    }
  }
  // 如果是左侧边界马路牙子
  if (hdmap::LeftBoundaryType(waypoint) == hdmap::LaneBoundaryType::CURB) {
    safe_lane_width -= smoother_config_.curb_shift();
    if (safe_lane_width < kEpislon) {
      ADEBUG << "lane width smaller than adc width and left curb shift";
      effective_width = kEpislon;
      is_lane_width_safe = false;
    } else {
      center_shift -= 0.5 * smoother_config_.curb_shift();
    }
  }
  // 安全余量
  //  apply buffer if possible
  const double buffered_width =
      safe_lane_width - 2.0 * smoother_config_.lateral_buffer(); // 0.2
  safe_lane_width =
      buffered_width < kEpislon ? safe_lane_width : buffered_width; // 安全车道宽度选择

  // 根据车道宽度做中心偏移
  // shift center depending on the road width
  if (is_lane_width_safe) {
    effective_width = 0.5 * safe_lane_width;
  }
  // 将参考点做中心偏移
  ref_point += left_vec * center_shift;
  anchor.path_point = ref_point.ToPathPoint(s);
  anchor.lateral_bound = common::math::Clamp(
      effective_width, smoother_config_.min_lateral_boundary_bound(), // 0.1
      smoother_config_.max_lateral_boundary_bound()); // 0.5
  return anchor;
}

// 根据路径情况生成锚点
void ReferenceLineProvider::GetAnchorPoints(
    const ReferenceLine &reference_line,
    std::vector<AnchorPoint> *anchor_points) const {
  CHECK_NOTNULL(anchor_points);
  const double interval = smoother_config_.max_constraint_interval();

  // 根据参考线的长度及采样间隔获取锚点个数
  int num_of_anchors =
      std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  // 将参考线length按照anchors点数等距离分割，添加anchor_s
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                              &anchor_s);
  // 循环遍历anchor_s
  for (const double s : anchor_s) {
    // 根据anchor_s获取锚点，添加横纵向边界
    AnchorPoint anchor = GetAnchorPoint(reference_line, s);
    // 添加锚点
    anchor_points->emplace_back(anchor);
  }
  // 起始和终止锚点横纵向边界，强制约束
  anchor_points->front().longitudinal_bound = 1e-6;
  anchor_points->front().lateral_bound = 1e-6;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}

// 数据转换为标准的referenceline，然后平滑
bool ReferenceLineProvider::SmoothRouteSegment(const RouteSegments &segments,
                                               ReferenceLine *reference_line) {
  // 查看path构造函数
  hdmap::Path path(segments);
  // 进入referenceline构造函数查看ReferenceLine构造
  return SmoothReferenceLine(ReferenceLine(path), reference_line);
}

bool ReferenceLineProvider::SmoothPrefixedReferenceLine(
    const ReferenceLine &prefix_ref, const ReferenceLine &raw_ref,
    ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_ref;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_ref, &anchor_points);
  // modify anchor points based on prefix_ref
  for (auto &point : anchor_points) {
    common::SLPoint sl_point;
    if (!prefix_ref.XYToSL(point.path_point, &sl_point)) {
      continue; // 如果投影失败则跳过
    }
    if (sl_point.s() < 0 || sl_point.s() > prefix_ref.Length()) {
      continue; // 如果投影s小于0，说明在参考线后面，如果大于prefix_ref.Length()，则说明在参考线前面，都直接跳过
    }
    auto prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s());
    point.path_point.set_x(prefix_ref_point.x());
    point.path_point.set_y(prefix_ref_point.y());
    point.path_point.set_z(0.0);
    point.path_point.set_theta(prefix_ref_point.heading());
    point.longitudinal_bound = 1e-6;
    point.lateral_bound = 1e-6;
    point.enforced = true;  // 如果在这之前，说明这些点已经被优化过了，就不需要优化了，那么直接绑定到原来的点就可以了
    break;
  }

  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_ref, reference_line)) {
    AERROR << "Failed to smooth prefixed reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_ref, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}

bool ReferenceLineProvider::SmoothReferenceLine(
    const ReferenceLine &raw_reference_line, ReferenceLine *reference_line) {
  // 如果不使用优化，则直接将参考线进行赋值
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_reference_line;
    return true;
  }
  // 生成锚点
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  // 获取锚点
  GetAnchorPoints(raw_reference_line, &anchor_points);
  // 给smooth_添加锚点信息，这里smoother_为DistancePointsReferenceLineSmoother
  smoother_->SetAnchorPoints(anchor_points);
  // 参考线优化
  if (!smoother_->Smooth(raw_reference_line, reference_line)) {
    AERROR << "Failed to smooth reference line with anchor points";
    return false;
  }
  // 优化参考线是否有效，就是利用优化参考线上的点与原始参考线计算l值，如果偏离了5米
  // 则认为无效
  if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}

// debug
bool ReferenceLineProvider::GetAdcWaypoint(
    hdmap::LaneWaypoint *waypoint) const {
  if (nullptr == current_pnc_map_) {
    AERROR << "Cannot find pnc map to get adc waypoint!";
    return false;
  }
  *waypoint = current_pnc_map_->GetAdcWaypoint();
  return true;
}

// debug
bool ReferenceLineProvider::GetAdcDis2Destination(double *dis) const {
  if (nullptr == current_pnc_map_) {
    AERROR << "Cannot find pnc map to get adc distance to destination!";
    return false;
  }
  *dis = current_pnc_map_->GetDistanceToDestination();
  return true;
}

}  // namespace planning
}  // namespace apollo
