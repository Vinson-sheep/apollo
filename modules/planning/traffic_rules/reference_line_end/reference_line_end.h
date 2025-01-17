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

#include "modules/planning/traffic_rules/reference_line_end/proto/reference_line_end.pb.h"

#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"
namespace apollo {
namespace planning {

// 当参考线结束时，一般需要重新路由，所以需要停车，这种情况下如果程序正常，一般是前方没有路了，
// 需要重新查询当前点到目的地新的路由
/**
 * This class decides whether we should send rerouting request based on traffic
 * situation.
 */
class ReferenceLineEnd : public TrafficRule {
 public:
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~ReferenceLineEnd() = default;
  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  void Reset() override {}

 private:
  ReferenceLineEndConfig config_;
  static constexpr char const* REF_LINE_END_VO_ID_PREFIX = "REF_END_";
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ReferenceLineEnd,
                                     TrafficRule)

}  // namespace planning
}  // namespace apollo
