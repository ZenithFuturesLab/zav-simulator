#pragma once
#include <epic_ros2/ros.hpp>
#include <omp.h>
#include <algorithm>
#include <optional>

namespace epic_ros {
// An explicit onboard setting applies to every planning loop. If absent,
// preserve the worker count each stage used in the reference flights.
inline int planningWorkerCount(int stage_default = 2) {
  static const std::optional<int> configured = []() -> std::optional<int> {
    NodeHandle nh;
    if (!nh.native()->has_parameter("exploration.planning_workers")) return std::nullopt;
    int workers;
    nh.getParam("exploration/planning_workers", workers);
    return workers;
  }();
  return std::clamp(configured.value_or(stage_default), 1, omp_get_num_procs());
}
}
