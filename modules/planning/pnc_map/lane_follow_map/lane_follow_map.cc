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
 * @file lane_follow_map.cc
 **/

#include "modules/planning/pnc_map/lane_follow_map/lane_follow_map.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "google/protobuf/text_format.h"

#include "modules/common_msgs/map_msgs/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::VehicleState;
using apollo::common::util::PointFactory;
using apollo::routing::RoutingResponse;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace

LaneFollowMap::LaneFollowMap() : hdmap_(hdmap::HDMapUtil::BaseMapPtr()) {}

bool LaneFollowMap::CanProcess(const planning::PlanningCommand &command) const {
  return command.has_lane_follow_command();
}

hdmap::LaneWaypoint LaneFollowMap::ToLaneWaypoint(
    const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return hdmap::LaneWaypoint(lane, waypoint.s());
}

hdmap::LaneSegment LaneFollowMap::ToLaneSegment(
    const routing::LaneSegment &segment) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return hdmap::LaneSegment(lane, segment.start_s(), segment.end_s());
}

void LaneFollowMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }
  // Search backwards when the car is driven backward on the route.
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index >
             cur_index) {
    --next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index ==
             cur_index &&
         adc_waypoint_.s <
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  // Search forwards
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index <
             cur_index) {
    ++next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index ==
             routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >=
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

std::vector<routing::LaneWaypoint> LaneFollowMap::FutureRouteWaypoints() const {
  const auto &waypoints =
      last_command_.lane_follow_command().routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

void LaneFollowMap::GetEndLaneWayPoint(
  // 从命令中提取lane_waypoints
    std::shared_ptr<routing::LaneWaypoint> &end_point) const {
  if (!last_command_.has_lane_follow_command() ||
      !last_command_.lane_follow_command().has_routing_request()) {
    end_point = nullptr;
    return;
  }
  const auto &routing_request =
      last_command_.lane_follow_command().routing_request();
  if (routing_request.waypoint().size() < 1) {
    end_point = nullptr;
    return;
  }
  end_point = std::make_shared<routing::LaneWaypoint>();
  end_point->CopyFrom(*(routing_request.waypoint().rbegin()));
}

hdmap::LaneInfoConstPtr LaneFollowMap::GetLaneById(const hdmap::Id &id) const {
  if (nullptr == hdmap_) {
    return nullptr;
  }
  return hdmap_->GetLaneById(id);
}

bool LaneFollowMap::IsValid(const planning::PlanningCommand &command) const {
  if (!CanProcess(command)) {
    return false;
  }
  const auto &routing = command.lane_follow_command();
  const int num_road = routing.road_size();
  if (num_road == 0) {
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

void LaneFollowMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear();  // 清空
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    // 如果lane_id在range_lane_ids_中，说明成环，那么直接退出循环
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    // 插入lane_id
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

bool LaneFollowMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  // 判断routing是否有效
  if (!IsValid(last_command_)) {
    AERROR << "The routing is invalid when updating vehicle state.";
    route_segments_lane_ids_.clear();
    return false;
  }
  // 判断前后帧之间车辆位置距离是否超过阈值
  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) >
       FLAGS_replan_lateral_distance_threshold +
           FLAGS_replan_longitudinal_distance_threshold)) {
    // Position is reset, but not replan.
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  // 记录车辆状态数据 
  adc_state_ = vehicle_state;

  // 根据车辆位置获取adc_waypoint_，就是当前车辆位置在最近车道上的投影点
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    AERROR << "Failed to get waypoint from routing with point: " << "("
           << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    route_segments_lane_ids_.clear();
    return false;
  }

  // 根据adc_waypoint获取在route_index的索引号，也就是route_indices_索引
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }
  ADEBUG << "adc_waypoint_" << adc_waypoint_.DebugString() << "route_index"
         << route_index;

  // 根据当前车辆位置判断route_index，根据route_index判断下一个routing waypoint索引
  // Track how many routing request waypoints the adc have passed.
  UpdateNextRoutingWaypointIndex(route_index);

  // 记录当前帧index，给下一帧使用
  adc_route_index_ = route_index;

  // 更新range_lane_ids
  UpdateRoutingRange(adc_route_index_);

  if (routing_waypoint_index_.empty()) {
    AERROR << "No routing waypoint index.";
    return false;
  }

  // 最后一个点，设置目的地停车标志位为true
  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) {
    stop_for_destination_ = true;
  }
  return true;
}

// 响应routing命令
// command中包含了多条routing路径
bool LaneFollowMap::UpdatePlanningCommand(
    const planning::PlanningCommand &command) {
  if (!CanProcess(command)) {
    AERROR << "Command cannot be processed by LaneFollowMap!";
    return false;
  }
  if (!PncMapBase::UpdatePlanningCommand(command)) {
    return false;
  }
  const auto &routing = command.lane_follow_command();
  // 清除数据
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();
  // routing响应给了几条road，每条road有几个passage，每个passage有几个lane
  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        // 获取所有lane_id
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        route_indices_.back().segment =
            ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        // 赋值road index, passage index, lane index
        // route_indices很重要
        route_indices_.back().index = {road_index, passage_index, lane_index};
      }
    }
  }

  range_start_ = 0;
  range_end_ = 0;

  // 根据adc_route_index_来确定车辆在哪个road 哪个passage 哪个lane
  // 参考route_indices_[adc_route_index_]
  adc_route_index_ = -1;

  // 下一个routing waypoint索引，参考变量定义，参考route_indices_索引
  next_routing_waypoint_index_ = 0;

  // 根据adc_route_index更新车道range_lane_ids
  UpdateRoutingRange(adc_route_index_);

  // routing request waypoint在route indices_索引
  routing_waypoint_index_.clear();
  // 获取routing request waypoints
  const auto &request_waypoints = routing.routing_request().waypoint();
  // 如果request waypoints为空
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }

  // 外层循环遍历route_indices_, 内层循环遍历request_waypoints，如果request_waypoints
  // 在route_indices_所在的lane上，添加routing_waypoint_index_信息，
  int i = 0;
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    // 其实是找到路径点所在的Lane索引
    while (i < request_waypoints.size() &&
           hdmap::RouteSegments::WithinLaneSegment(route_indices_[j].segment,
                                                   request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(
          hdmap::LaneWaypoint(route_indices_[j].segment.lane,
                              request_waypoints.Get(i).s()),
          j);
      ++i;
    }
  }
  adc_waypoint_ = hdmap::LaneWaypoint();
  stop_for_destination_ = false;
  return true;
}

int LaneFollowMap::SearchForwardWaypointIndex(
    int start, const hdmap::LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (i < static_cast<int>(route_indices_.size()) &&
         !hdmap::RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                  waypoint)) {
    ++i;
  }
  return i;
}

int LaneFollowMap::SearchBackwardWaypointIndex(
    int start, const hdmap::LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !hdmap::RouteSegments::WithinLaneSegment(
                       route_indices_[i].segment, waypoint)) {
    --i;
  }
  return i;
}

int LaneFollowMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int LaneFollowMap::GetWaypointIndex(const hdmap::LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

// 将车道上的车道段全部提取
bool LaneFollowMap::PassageToSegments(routing::Passage passage,
                                      hdmap::RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  for (const auto &lane : passage.segment()) {
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

// 获取road对应的passage
// 只考虑当前车道和左右车道
std::vector<int> LaneFollowMap::GetNeighborPassages(
    const routing::RoadSegment &road, int start_passage) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  std::vector<int> result;
  // 获取start_passage (当前车辆所在的passage)对应的passage内容
  const auto &source_passage = road.passage(start_passage);
  // 添加到result
  result.emplace_back(start_passage);
  // 如果当前车辆的passage为直行道路，无法变道，则直接返回
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }
  // 如果当前车辆的passage准备退出，车辆已经准备进入下一个passage，不需要变道
  if (source_passage.can_exit()) {  // No need to change lane
    return result;
  }
  // 根据当前车道所在的passage提取std::vector<LaneSegment>也是source_segments
  hdmap::RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return result;
  }
  // 如果下一个routing waypoint查询点在当前车辆所在的通道中。不需要变道，直接返回
  // 车辆所在的passage
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(
          routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
    return result;
  }
  std::unordered_set<std::string> neighbor_lanes;
  // 如果车辆可以左变道或者右变道，从高精度地图map中查询当前车道左侧和右侧所有车道线
  // 如果去和RoadSegment()作比较，就找到两者共同包含的车道，就是最终的临近车道
  // 如果当前passage的变道类型为左变道
  if (source_passage.change_lane_type() == routing::LEFT) {
    for (const auto &segment : source_segments) {
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } 
  // 如果当前passage的变道类型为右变道
  else if (source_passage.change_lane_type() == routing::RIGHT) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }
  // 提取对比结果
  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i);
        break;
      }
    }
  }
  return result;
}
bool LaneFollowMap::GetRouteSegments(
    const VehicleState &vehicle_state,
    std::list<hdmap::RouteSegments> *const route_segments) {
  // 前视距离 根据当前车速 * 时距(8s) > 180m ? 250 : 180
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = FLAGS_look_backward_distance;
  // 后视距离 50m
  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments);
}

// 最核心的函数
bool LaneFollowMap::GetRouteSegments(
    const VehicleState &vehicle_state, const double backward_length,
    const double forward_length,
    std::list<hdmap::RouteSegments> *const route_segments) {
  // 首先获取车辆状态
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  // 如果车辆状态异常，则不规划
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }
  // 根据当前车辆位置在route_index的索引来获取road passage lane信息
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0];
  const int passage_index = route_index[1];
  const auto &road = last_command_.lane_follow_command().road(road_index);

  // Raw filter to find all neighboring passages
  // 根据当前车辆所在的passage去获取临近的passage
  // 这个过程筛选出可以通行的车道
  // 只考虑当前车道和左右车道
  auto drive_passages = GetNeighborPassages(road, passage_index);

  // 在上一步中，找到了当前车辆在规划轨迹中的邻近车道，这一步就对每一个车道做一个是否可驶入的检查，
  // 并作道路段截取，也就是制定出当前车辆在当前情况下可能驶入的区域。每个passage将划分出一个道路区间，
  // 道路区间的长度由前向距离查询（看函数输入）决定，短期内可行驶道路长度为forward_length + 
  // backward_length。
  
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index);
    hdmap::RouteSegments segments;
    // 不能根据passage提取车道信息，则跳过
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
    // 如果index为当前车辆所在的passage，那么nearest_point提取根据当前车辆的waypoint.s计算得到
    // 否则直接将当前车辆的位置转换为nearest_point
    const PointENU nearest_point =
        index == passage_index
            ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
            : PointFactory::ToPointENU(adc_state_);
    // 根据nearest_point计算在passage上的sl和segment_waypoint，也就是获取了
    // 当前车辆位置在passage上的投影信息
    common::SLPoint sl;
    hdmap::LaneWaypoint segment_waypoint;
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
    // 如果index不等于当前车辆所在passage，做一个是否可驶入的检查
    if (index != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }

    // 获取route_segments，并添加信息
    route_segments->emplace_back();
    // 获取LaneSegment最后一个waypoint
    const auto last_waypoint = segments.LastWaypoint();

    // 这部分就是对上述通过可驶入合法性检查的车道进行道路段的生成，同时使用backward_length
    // 和forward_length前后扩展道路段，从代码可以看出，车辆在passage中的投影点累积距离为sl.s
    // (注意这个s是投影在passage段起点的累积距离，并非整个road的累积距离)，具体计算参考GetProjection函数
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }
    // 记录route_segments的最后一个waypoint点
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    // passage能否退出
    route_segments->back().SetCanExit(passage.can_exit());
    // 设置下一个行动标识，也就是变道类型
    route_segments->back().SetNextAction(passage.change_lane_type());
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments->back().SetId(route_segment_id);
    // 添加目的地停车标识
    route_segments->back().SetStopForDestination(stop_for_destination_);
    // 如果当前index是当前车辆所在标识，则设置标识位为true
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(routing::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(routing::LEFT);
    }
  }
  UpdateRouteSegmentsLaneIds(route_segments);
  return !route_segments->empty();
}

// 根据当前车辆位置获取adc_waypoint，就是当前车辆在最近车道线上的投影点
bool LaneFollowMap::GetNearestPointFromRouting(
    const common::VehicleState &state, hdmap::LaneWaypoint *waypoint) const {
  waypoint->lane = nullptr;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  // 转换为ENU格式
  const auto point = PointFactory::ToPointENU(state);
  // 获取valid lanes
  std::vector<hdmap::LaneInfoConstPtr> valid_lanes;
  for (auto lane_id : all_lane_ids_) {
    hdmap::Id id = hdmap::MakeMapId(lane_id);
    auto lane = hdmap_->GetLaneById(id);
    if (nullptr != lane) {
      valid_lanes.emplace_back(lane);
    }
  }

  // 根据当前车辆位置获取最近的waypoint点
  // Get nearest_waypoints for current position
  std::vector<hdmap::LaneWaypoint> valid_way_points;
  for (const auto &lane : valid_lanes) {  
    // 过滤无效车道
    if (range_lane_ids_.count(lane->id().id()) == 0) {
      ADEBUG << "not in range" << lane->id().id();
      continue;
    }
    if (route_segments_lane_ids_.size() > 0 &&
        route_segments_lane_ids_.count(lane->id().id()) == 0) {
      ADEBUG << "not in last frame route_segments: " << lane->id().id();
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    {
      // 根据当前车辆位置计算车辆在lane上的s值和l值
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        continue;
      }
      ADEBUG << lane->id().id() << "," << s << "," << l;
      // 根据计算的s值过滤无效车道
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
      double lane_heading = lane->Heading(s);
      if (std::fabs(common::math::AngleDiff(lane_heading, state.heading())) >
          M_PI_2 * 1.5) {
        continue;
      }
    }
    // 填充有效waypoint信息，可能有几条车道符合上述条件
    valid_way_points.emplace_back();
    auto &last = valid_way_points.back();
    last.lane = lane;
    last.s = s;
    last.l = l;
    ADEBUG << "distance:" << std::fabs(l);
  }
  // 如果有效waypoint信息为空，则返回错误
  if (valid_way_points.empty()) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
    return false;
  }

  // 如果超过一个候选车道线，选择正确航向角度的车道线，如果没有正确航向角度的车道线，
  // 则选择最近的车道线
  // find closest lane that satisfy vehicle heading
  int closest_index = -1;
  double distance = std::numeric_limits<double>::max();
  double lane_heading = 0.0;
  double vehicle_heading = state.heading();
  for (size_t i = 0; i < valid_way_points.size(); i++) {
    // 根据当前车道航向角度和车辆航向角度判断
    lane_heading = valid_way_points[i].lane->Heading(valid_way_points[i].s);
    if (std::abs(common::math::AngleDiff(lane_heading, vehicle_heading)) >
        M_PI_2 * 1.5) {
      continue;
    }
    // 获取最近的参考线
    if (std::fabs(valid_way_points[i].l) < distance) {
      distance = std::fabs(valid_way_points[i].l);
      closest_index = i;
    }
  }
  if (closest_index == -1) {
    AERROR << "Can not find nearest waypoint. vehicle heading:"
           << vehicle_heading << "lane heading:" << lane_heading;
    return false;
  }
  // 赋值最近位点，即 lane, s, l
  waypoint->lane = valid_way_points[closest_index].lane;
  waypoint->s = valid_way_points[closest_index].s;
  waypoint->l = valid_way_points[closest_index].l;
  return true;
}

hdmap::LaneInfoConstPtr LaneFollowMap::GetRouteSuccessor(
    hdmap::LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

hdmap::LaneInfoConstPtr LaneFollowMap::GetRoutePredecessor(
    hdmap::LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (const auto &route_index : route_indices_) {
    auto &lane = route_index.segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

bool LaneFollowMap::ExtendSegments(const hdmap::RouteSegments &segments,
                                   const common::PointENU &point,
                                   double look_backward, double look_forward,
                                   hdmap::RouteSegments *extended_segments) {
  common::SLPoint sl;
  hdmap::LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

// 如果长度不够，则延长；否则，截断
bool LaneFollowMap::ExtendSegments(
    const hdmap::RouteSegments &segments, double start_s, double end_s,
    hdmap::RouteSegments *const truncated_segments) const {
  // 如果segments为空
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  // 将segments的属性信息赋值给truncated_segments
  truncated_segments->SetProperties(segments);
  // 如果起始s大于等于终点s则返回错误
  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;
  // 当前车道投影点所在车道的后向查询起始点小于0，即sl.s - backward_length < 0 
  // 也就是此时起始距离已经落后第一条车道，此时就需要查询first lane segment，即
  // segments第一条车道前置部分进行截取。如果前置部分仍然不够长度sl.s + backward_length，
  // 那么就需要加入这条车道的前置车道进行截取 （也就是前置车道的前置车道）
  // sl.s是投影点相对于passage的累积距离
  // 而不是相对于整个规划路径road的累积距离。所以用小于0来判断后向查询点是否在第一个laneSegment之前
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) { 
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane; // 获取第一条车道
    double s = first_segment.start_s; // 获取第一条车道起始s
    double extend_s = -start_s;   // extend_s为需要从牵扯道中截取的道路段长度，初始化为-start_s
    std::vector<hdmap::LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {  // 每次循环（截取）以后extend_s都会减小，直到为0
      if (s <= kRouteEpsilon) { // s小于等于0，则需要查询这条lane对应的前置车道进行截取
        lane = GetRoutePredecessor(lane);
        if (lane == nullptr ||
            unique_lanes.find(lane->id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else {  // 如果s大于0，此时就可以从这条前置lane中截取路段，或者刚开始first_segment.start_s大于0，那么继续从第一条车道截取
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length; // 更新extend_s，如果extend_s > 0，说明还需要继续寻找前置道路段截取
        s -= length;
        unique_lanes.insert(lane->id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }

  // 正常passage的laneSegment截取
  bool found_loop = false;
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() ==
              lane_segment.lane->id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->id().id()) ==
                 unique_lanes.end()) {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      } else {
        found_loop = true;  // 说明这个导航路径是闭环的，那么直接退出
        break;
      }
    }
    router_s += (lane_segment.end_s - lane_segment.start_s);
    // 退出循环条件
    if (router_s > end_s) {
      break;
    }
  }
  // 如果导航路径是闭环的，则直接返回true
  if (found_loop) {
    return true;
  }
  // Extend the trajectory towards the end of the trajectory.
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) {
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }
  auto last_lane = segments.back().lane;
  while (router_s < end_s - kRouteEpsilon) {
    last_lane = GetRouteSuccessor(last_lane);
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->id().id()) != unique_lanes.end()) {
      break;
    }
    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->id().id());
    router_s += length;
  }
  return true;
}

void LaneFollowMap::AppendLaneToPoints(
    hdmap::LaneInfoConstPtr lane, const double start_s, const double end_s,
    std::vector<hdmap::MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           hdmap::LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points->emplace_back(segment.start() + segment.unit_direction() *
                                                   (start_s - accumulate_s),
                             lane->headings()[i],
                             hdmap::LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], hdmap::LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

void LaneFollowMap::UpdateRouteSegmentsLaneIds(
    const std::list<hdmap::RouteSegments> *route_segments) {
  route_segments_lane_ids_.clear();
  for (auto &route_seg : *route_segments) {
    for (auto &lane_seg : route_seg) {
      if (nullptr == lane_seg.lane) {
        continue;
      }
      route_segments_lane_ids_.insert(lane_seg.lane->id().id());
    }
  }
}

apollo::hdmap::LaneWaypoint LaneFollowMap::GetAdcWaypoint() const {
  return adc_waypoint_;
}

double LaneFollowMap::GetDistanceToDestination() const {
  if (adc_route_index_ < 0 || adc_route_index_ >= route_indices_.size()) {
    AERROR << "adc_route_index error, can not get distance to destination, "
              "return 0.";
    return 0.0;
  }
  const auto &routing = last_command_.lane_follow_command();
  int adc_road_index = route_indices_[adc_route_index_].index[0];
  int adc_passage_index = route_indices_[adc_route_index_].index[1];
  int adc_lane_index = route_indices_[adc_route_index_].index[2];

  bool get_adc_exit_waypoint =
      routing.road(adc_road_index).passage(adc_passage_index).can_exit();
  int start_passage_index = adc_passage_index;
  int start_lane_index = adc_lane_index;
  double start_lane_s = adc_waypoint_.s;

  double dis_to_destination = 0.0;

  for (int road_index = adc_road_index; road_index < routing.road_size();
       ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      if (!passage.can_exit()) {
        continue;
      }
      // check this passsage is can_exit
      if (!get_adc_exit_waypoint) {
        get_adc_exit_waypoint = true;
        // find adc waypoint in passage which can exit
        for (int index = 0; index < passage.segment_size(); ++index) {
          auto lane = hdmap_->GetLaneById(
              hdmap::MakeMapId(passage.segment(index).id()));
          double s = 0.0;
          double l = 0.0;
          if (!lane->GetProjection({adc_state_.x(), adc_state_.y()}, &s, &l)) {
            continue;
          }
          static constexpr double kEpsilon = 0.5;
          if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
            continue;
          }
          start_passage_index = passage_index;
          start_lane_index = index;
          start_lane_s = s;
        }
      }

      // add distance by lane in can_exit passage
      if (get_adc_exit_waypoint) {
        if (road_index == adc_road_index &&
            passage_index == start_passage_index) {
          for (int lane_index = start_lane_index;
               lane_index < passage.segment_size(); ++lane_index) {
            if (lane_index == start_lane_index) {
              dis_to_destination +=
                  passage.segment(lane_index).end_s() - start_lane_s;
            } else {
              dis_to_destination += passage.segment(lane_index).end_s() -
                                    passage.segment(lane_index).start_s();
            }
          }
        } else {
          for (int lane_index = 0; lane_index < passage.segment_size();
               ++lane_index) {
            dis_to_destination += passage.segment(lane_index).end_s() -
                                  passage.segment(lane_index).start_s();
          }
        }
        break;
      }
    }
  }
  return dis_to_destination;
}

}  // namespace planning
}  // namespace apollo
