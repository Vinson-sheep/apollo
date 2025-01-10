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

#pragma once

#include <memory>
#include <string>

#include "modules/planning/traffic_rules/keepclear/proto/keepclear.pb.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

// 禁停区分成两类，第一类是传统的禁停区，第二类是交叉路口。对于禁停区的处理和对人行横道上障碍物构建虚拟墙很相似。
// 具体做法是在参考线上构建一块禁停区，从纵向的start_s到end_s（这里的start_s和end_s是禁停区start_s和end_s
// 在参考线上的投影点），禁停区宽度在配置文件中设置(4米)
/**
 * This class creates a virtual obstacle for each clear area region.
 */
class KeepClear : public TrafficRule {
 public:
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~KeepClear() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  void Reset() override {}

 private:
  bool IsCreeping(const double pnc_junction_start_s,
                  const double adc_front_edge_s) const;

  bool BuildKeepClearObstacle(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info,
                              const std::string& virtual_obstacle_id,
                              const double keep_clear_start_s,
                              const double keep_clear_end_s);

 private:
  KeepClearConfig config_;
  static constexpr char const* KEEP_CLEAR_VO_ID_PREFIX = "KC_";
  static constexpr char const* KEEP_CLEAR_JUNCTION_VO_ID_PREFIX = "KC_JC_";
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::KeepClear, TrafficRule)

}  // namespace planning
}  // namespace apollo
