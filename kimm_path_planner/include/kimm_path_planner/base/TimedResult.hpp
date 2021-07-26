#pragma once

#include <ompl/base/Planner.h>

#include "kimm_path_planner/utils/Stopwatch.hpp"

namespace ob = ompl::base;

class TimedResult : public Stopwatch {
 public:
  ompl::geometric::PathGeometric trajectory;
  ob::PlannerStatus status;

  explicit TimedResult(
      const ompl::geometric::PathGeometric &trajectory =
          ompl::geometric::PathGeometric(global::settings.ompl.space_info),
      double time = 0)
      : trajectory(trajectory) {
    this->elapsed_ = time;
  }
};
