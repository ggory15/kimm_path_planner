#pragma once

#include <ompl/geometric/PathGeometric.h>
#include <algorithm>

#include "kimm_path_planner/planners/AbstractPlanner.h"

#include "gnode.h"
#include "stl_thetastar.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ThetaStar : public AbstractPlanner, public ob::Planner {
 public:
  ThetaStar();
  virtual ~ThetaStar();

  std::string name() const override { return "Theta*"; }

  bool initialize();

  void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;
  ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

  ob::PlannerStatus run() override;

  og::PathGeometric solution() const override;

  bool hasReachedGoalExactly() const override;
  double planningTime() const override;
  unsigned int steps() const;

 private:
  bool COST_SEARCH;
  bool USE_ASTAR;
  bool USE_GRANDPARENT;

  double _planningTime;
  unsigned int _steps;

  std::vector<std::vector<GNode> > global_paths;

  bool search(std::vector<std::vector<GNode> > &paths, GNode start, GNode goal);

 protected:
  explicit ThetaStar(bool astar, std::string name);

 public:
  inline ob::Planner *omplPlanner() override { return this; }
};
