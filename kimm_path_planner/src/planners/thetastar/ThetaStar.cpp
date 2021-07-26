#include <ompl/base/Planner.h>
#include <utility>

#include "kimm_path_planner/utils/PlannerUtils.hpp"
#include "kimm_path_planner/utils/Stopwatch.hpp"

#include "kimm_path_planner/planners/thetastar/ThetaStar.h"

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0
#define INTERACTIVE_STEP_THROUGH 0
#define STEP_DELAY 0
//#define TESTM 0

ThetaStar::ThetaStar()
    : AbstractPlanner(name()),
      ob::Planner(ss->getSpaceInformation(), "Theta*") {
  srand((unsigned int)(time(nullptr)));

  /// Euclidean Cost
  COST_SEARCH = true;

  // USE_ASTAR
  USE_ASTAR = false;

  _planningTime = 0;

  pdef_ = std::make_shared<ompl::base::ProblemDefinition>(
      ss->getSpaceInformation());
}

ThetaStar::ThetaStar(bool astar, std::string name)
    : AbstractPlanner(name),
      ob::Planner(ss->getSpaceInformation(), std::move(name)) {
  srand((unsigned int)(time(nullptr)));

  /// Euclidean Cost
  COST_SEARCH = true;

  // USE_ASTAR
  USE_ASTAR = astar;

  _planningTime = 0;

  pdef_ = std::make_shared<ompl::base::ProblemDefinition>(
      ss->getSpaceInformation());
}

ThetaStar::~ThetaStar() = default;

bool ThetaStar::initialize() { return true; }

bool ThetaStar::search(std::vector<std::vector<GNode> > &paths, GNode start,
                       GNode goal) {
  paths.clear();
  Stopwatch sw;
  sw.start();

  //    OMPL_DEBUG("Theta*: Start: %d, %d --- Goal: %d, %d ", start.x, start.y,
  //    goal.x, goal.y);

  std::vector<GNode> sol;
  std::vector<std::vector<GNode> > path_sol;

  ThetaStarSearch<GNode> thetastarsearch(COST_SEARCH);

  if (USE_ASTAR) thetastarsearch.useAstar();

  if (USE_GRANDPARENT) thetastarsearch.use_connectGrandParent();

  _steps = 0;

  const unsigned int NumSearches = 1;

  GNode *p, *lastOpen = nullptr;
  unsigned int SearchCount = 0;
  while (SearchCount < NumSearches) {
    // Set Start and goal states
    thetastarsearch.SetStartAndGoalStates(start, goal);

    unsigned int SearchState;
    do {
      if (sw.elapsed() > global::settings.max_planning_time) {
        OMPL_WARN("Theta* could not finish within the allotted time limit.");
        return false;
      }
      SearchState = thetastarsearch.SearchStep();

      _steps++;

      if (SearchState != ThetaStarSearch<GNode>::SEARCH_STATE_SEARCHING) break;

      p = thetastarsearch.GetOpenListStart();
      //      if (p == nullptr) OMPL_INFORM("THETA*: No open nodes");

#if DEBUG
      while (p) {
        QtVisualizer::drawNode(*p, Qt::darkYellow, 0.2, false);
        p = thetastarsearch.GetOpenListNext();
      }
#endif
#if DEBUG_LISTS
      OMPL_INFORM("Step: %d", (int)_steps);

      int len = 0;

      OMPL_INFORM("Open:");
      p = thetastarsearch.GetOpenListStart();
      lastOpen = p;
      if (p == nullptr)
        OMPL_INFORM("No open nodes");
      else
        _publisher.publishOpenNode(p, true);

      while (p) {
        len++;

#if !DEBUG_LIST_LENGTHS_ONLY
        // p->PrintNodeInfo();
        if (len > 1) _publisher.publishOpenNode(p);
#endif

        p = thetastarsearch.GetOpenListNext();
      }
      OMPL_INFORM("Open list has %d", len);

      len = 0;

      OMPL_INFORM("Closed");
      p = thetastarsearch.GetClosedListStart();
      while (p) {
        len++;

#if !DEBUG_LIST_LENGTHS_ONLY
        // p->PrintNodeInfo();
        _publisher.publishClosedNode(p);
#endif

        p = thetastarsearch.GetClosedListNext();
      }
      OMPL_INFORM("Closed list has %d nodes", len);
#endif

#if DEBUG_LISTS
      ThetaStarSearch<GNode>::Node *node =
          thetastarsearch
              .GetCurrentBestRawNode();  // thetastarsearch.GetSolutionStart();
      if (node == nullptr) continue;
      sol.push_back(GNode(node->m_UserState.x, node->m_UserState.y,
                          node->m_UserState.theta, node->m_UserState.steer,
                          node->m_UserState.steer_cost, node->m_UserState.costs,
                          node->m_UserState.orientations));
      bool repeating = false;
      //            _publisher.publish(_publisher._createNodeMarker(&(node->m_UserState),
      //            .1f, .7f, .1f));
      std::vector<GNode> reached;
      reached.push_back(node->m_UserState);
      while ((node = node->parent) && !repeating) {
        OMPL_INFORM("Found parent at %d %d", node->m_UserState.x,
                    node->m_UserState.y);
        for (GNode &r : reached) {
          if (r.x == node->m_UserState.x && r.y == node->m_UserState.y) {
            repeating = true;
            break;
          }
        }
        if (repeating) break;
        sol.push_back(GNode(node->m_UserState.x_r, node->m_UserState.y_r,
                            node->m_UserState.theta, node->m_UserState.steer,
                            node->m_UserState.steer_cost,
                            node->m_UserState.costs,
                            node->m_UserState.orientations));
        reached.push_back(node->m_UserState);
        //                pub_open_closed_.publish(_createNodeMarker(&(node->m_UserState),
        //                .1f, .7f, .1f));
      }

      _publisher.publishGlobalPath(sol, true);

      OMPL_INFORM("Generated partial path of length %d", (int)sol.size());
      //            if (sol.size() > 1)
      //                _publisher.publishGlobalTraj(sol, false, 1);

      sol.clear();
#endif

#if INTERACTIVE_STEP_THROUGH
      OMPL_INFORM("Press key to proceed");
      std::cin.get();
#endif

#if STEP_DELAY
      usleep(100000);  // in microseconds
#endif
    } while (SearchState == ThetaStarSearch<GNode>::SEARCH_STATE_SEARCHING);

    if (SearchState == ThetaStarSearch<GNode>::SEARCH_STATE_SUCCEEDED) {
      //            OMPL_DEBUG("Theta* search found goal state.");

      GNode *node = thetastarsearch.GetSolutionStart();
      int steps = 0;
      int xs, ys;

      xs = node->x;
      ys = node->y;
      sol.emplace_back(GNode(xs, ys, node->theta, node->steer, node->steer_cost,
                             node->costs, node->orientations));

      while ((node = thetastarsearch.GetSolutionNext())) {
        sol.emplace_back(*node);
        steps++;
      }

      // Once you're done with the solution you can free the nodes up
      thetastarsearch.FreeSolutionNodes();
    } else if (SearchState == ThetaStarSearch<GNode>::SEARCH_STATE_FAILED) {
      OMPL_ERROR("Theta* search terminated. Did not find goal state.");
    }

    // Display the number of loops the search went through
    OMPL_DEBUG("Theta* steps: %d ", (int)_steps);

    SearchCount++;

    thetastarsearch.EnsureMemoryFreed();
  }

  paths.push_back(sol);

  return !paths.empty() && !paths[0].empty();
}

ob::PlannerStatus ThetaStar::run() {
  ob::ScopedState<> start(ss->getStateSpace());
  start[0] = global::settings.environment->start().x;
  start[1] = global::settings.environment->start().y;
  start[2] = global::settings.environment->startTheta();
  ob::ScopedState<> goal(ss->getStateSpace());
  goal[0] = global::settings.environment->goal().x;
  goal[1] = global::settings.environment->goal().y;
  goal[2] = global::settings.environment->goalTheta();

  pdef_->setStartAndGoalStates(start, goal);

  return ob::Planner::solve(global::settings.max_planning_time);
}

og::PathGeometric ThetaStar::solution() const {
  og::PathGeometric path(ss->getSpaceInformation());
  double unit = global::settings.environment->unit();
  auto gnodes = global_paths[0];
  if (gnodes.empty()) {
    OMPL_ERROR("Theta*: The computed path contains no GNodes!");
    return path;
  }
  for (auto &node : gnodes) {
    auto *state =
        ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
    state->setXY(node.x_r * unit, node.y_r * unit);
    state->setYaw(node.theta);
    path.append(state);
  }
  return path;
}

bool ThetaStar::hasReachedGoalExactly() const { return true; }

double ThetaStar::planningTime() const { return _planningTime; }

void ThetaStar::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) {
  pdef_ = pdef;
}

ob::PlannerStatus ThetaStar::solve(const ob::PlannerTerminationCondition &ptc) {
  pdef_->clearSolutionPaths();
  pdef_->clearSolutionNonExistenceProof();

  auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
  double unit = global::settings.environment->unit();

  if (goal == nullptr) {
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
  }

  if (!goal->couldSample()) {
    OMPL_ERROR("%s: Insufficient states in sampleable goal region",
               getName().c_str());
    return ob::PlannerStatus::INVALID_GOAL;
  }

  auto *goalState = ss->getStateSpace()->allocState();
  goal->sampleGoal(goalState);
  GNode goalNode(goalState->as<ob::SE2StateSpace::StateType>()->getX() / unit,
                 goalState->as<ob::SE2StateSpace::StateType>()->getY() / unit,
                 goalState->as<ob::SE2StateSpace::StateType>()->getYaw());

  auto *startState =
      pdef_->getStartState(0)->as<ob::SE2StateSpace::StateType>();
  GNode startNode(startState->getX() / unit, startState->getY() / unit,
                  startState->getYaw());

  global_paths.clear();

  //  global::settings.steering->clearInternalData();
  // XXX scale collision model to prevent that all solutions are colliding
  //  const auto original_shape =
  //      global::settings.env.collision.robot_shape.value();
  //  if (global::settings.env.collision.collision_model ==
  //  robot::ROBOT_POLYGON)
  //    global::settings.env.collision.robot_shape.value().scale(1);

  OMPL_DEBUG("Theta*: Generate a new global path.");
  Stopwatch sw;
  sw.start();
  search(global_paths, startNode, goalNode);
  sw.stop();
  _planningTime = sw.elapsed();

  // revert collision shape
  //  global::settings.env.collision.robot_shape = original_shape;

  OMPL_INFORM("Theta*: Search finished.");
  if (global_paths.empty() || global_paths[0].empty()) {
    OMPL_WARN("Theta*: No path found.");
    return ob::PlannerStatus::TIMEOUT;
  }

  for (auto &gnodes : global_paths) {
    auto path(std::make_shared<og::PathGeometric>(ss->getSpaceInformation()));
    for (auto &node : gnodes)
      path->append(
          base::StateFromXYT(node.x_r * unit, node.y_r * unit, node.theta));
    pdef_->addSolutionPath(path, false, 0.0, getName());
  }

  return ob::PlannerStatus::EXACT_SOLUTION;
}

unsigned int ThetaStar::steps() const { return _steps; }
