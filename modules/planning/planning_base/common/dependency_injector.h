/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/history.h"
#include "modules/planning/planning_base/common/learning_based_data.h"
#include "modules/planning/planning_base/common/planning_context.h"

namespace apollo {
namespace planning {

class DependencyInjector {
 public:
  DependencyInjector() = default;
  ~DependencyInjector() = default;

  // 负责planning上下文缓存，比如是否出发重新路由的ReroutingStatus信息，查看Proto
  PlanningContext* planning_context() { return &planning_context_; }

  // 是一个可索引队列，负责planning的输入、输出等主要信息的缓存，以frame类进行组织，
  // 内部包含localview结构体（负责输入数据的融合管理）。与history不同的是，
  // 该缓存数据自模块启动后就开始缓存所有的Frame对象，不受routing变动的影响
  FrameHistory* frame_history() { return &frame_history_; }
  // 负责障碍物状态的缓存，包括运动状态、决策结果。该数据与routing结果绑定，routing变动后会清理掉历史数据
  History* history() { return &history_; }
  // 提供车辆动、静信息，即车辆运动状态参数（轨迹、速度、加速度等）和车辆结构参数（长宽高等）
  EgoInfo* ego_info() { return &ego_info_; }
  // 车辆状态提供器，用于获取车辆实时信息
  apollo::common::VehicleStateProvider* vehicle_state() {
    return &vehicle_state_;
  }
  // 基于学习的数据，用于学习建模等
  LearningBasedData* learning_based_data() { return &learning_based_data_; }

 private:
  PlanningContext planning_context_;
  FrameHistory frame_history_;
  History history_;
  EgoInfo ego_info_;
  apollo::common::VehicleStateProvider vehicle_state_;
  LearningBasedData learning_based_data_;
};

}  // namespace planning
}  // namespace apollo
