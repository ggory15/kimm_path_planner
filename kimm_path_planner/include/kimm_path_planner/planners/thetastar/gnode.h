#pragma once

#include <ompl/util/Console.h>

#include "kimm_path_planner/base/PlannerSettings.h"
#include "kimm_path_planner/utils/PlannerUtils.hpp"

#include "gnode_base.h"
#include "stl_thetastar.h"

// Gnode class which includes useful methods for the Thetastar search
class GNode : public GNode_base {
 public:
  /// Different constructors
  GNode() {
    x = 0;
    y = 0;
    theta = 0;
    steer_cost = 0;
    steer = false;

    nEdges = global::settings.thetaStar.number_edges;
    READ_OR = 0;
    CHECK_SUCCESSOR = 0;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      costs[i] = 0;

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = 0;

    goal_x = (int)global::settings.environment->goal().x;
    goal_y = (int)global::settings.environment->goal().y;
    start_x = (int)global::settings.environment->start().x;
    start_y = (int)global::settings.environment->start().y;
    hasParent = false;
    parent = new GNode_base();
    child = new GNode_base();
  }

  GNode(double xx, double yy) {
    x = (int)xx;
    y = (int)yy;

    theta = 0;
    steer_cost = 0;
    steer = false;
    nEdges = global::settings.thetaStar.number_edges;
    READ_OR = 0;
    CHECK_SUCCESSOR = 0;
    x_r = xx;
    y_r = yy;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      costs[i] = 1000;

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = 0;

    goal_x = (int)global::settings.environment->goal().x;
    goal_y = (int)global::settings.environment->goal().y;
    start_x = (int)global::settings.environment->start().x;
    start_y = (int)global::settings.environment->start().y;
    hasParent = false;
    parent = new GNode_base();
    child = new GNode_base();
  }

  GNode(double xx, double yy, double tt) {
    x = (int)xx;
    y = (int)yy;
    theta = tt;

    steer_cost = 0;
    steer = false;
    nEdges = global::settings.thetaStar.number_edges;
    READ_OR = 0;
    CHECK_SUCCESSOR = 0;
    x_r = xx;
    y_r = yy;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      costs[i] = 1000;

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = 0;

    goal_x = (int)global::settings.environment->goal().x;
    goal_y = (int)global::settings.environment->goal().y;
    start_x = (int)global::settings.environment->start().x;
    start_y = (int)global::settings.environment->start().y;
    hasParent = false;
    parent = new GNode_base();
    child = new GNode_base();
  }

  GNode(double xx, double yy, double tt, int type) {
    x = (int)xx;
    y = (int)yy;
    theta = tt;

    steer_cost = 0;
    steer = type;
    nEdges = global::settings.thetaStar.number_edges;
    READ_OR = 0;
    CHECK_SUCCESSOR = 0;
    x_r = xx;
    y_r = yy;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      costs[i] = 1000;

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = 0;

    parent = new GNode_base();
    child = new GNode_base();

    goal_x = (int)global::settings.environment->goal().x;
    goal_y = (int)global::settings.environment->goal().y;
    start_x = (int)global::settings.environment->start().x;
    start_y = (int)global::settings.environment->start().y;
    hasParent = false;
  }

  GNode(double xx, double yy, double tt, int type, double steer_c) {
    x = (int)xx;
    y = (int)yy;
    theta = tt;

    steer_cost = steer_c;
    steer = type;
    nEdges = global::settings.thetaStar.number_edges;
    READ_OR = 0;
    CHECK_SUCCESSOR = 0;
    x_r = xx;
    y_r = yy;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      costs[i] = 1000;

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = 0;

    parent = new GNode_base();
    child = new GNode_base();

    goal_x = (int)global::settings.environment->goal().x;
    goal_y = (int)global::settings.environment->goal().y;
    start_x = (int)global::settings.environment->start().x;
    start_y = (int)global::settings.environment->start().y;
    hasParent = false;
  }

  GNode(double xx, double yy, double tt, int type, double steer_c, double *c,
        double *orien) {
    x = (int)xx;
    y = (int)yy;
    theta = (tt);

    steer_cost = steer_c;
    steer = type;
    nEdges = global::settings.thetaStar.number_edges;
    READ_OR = 0;
    CHECK_SUCCESSOR = 0;
    x_r = xx;
    y_r = yy;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++) {
      costs[i] = c[i];
    }

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = orien[i];

    parent = new GNode_base();
    child = new GNode_base();

    goal_x = (int)global::settings.environment->goal().x;
    goal_y = (int)global::settings.environment->goal().y;
    start_x = (int)global::settings.environment->start().x;
    start_y = (int)global::settings.environment->start().y;
    hasParent = false;
  }

  GNode(double xx, double yy, double tt, int type, double steer_c, double *c,
        double *orien, double xxx, double yyy) {
    hasParent = false;
    x = (int)xx;
    y = (int)yy;
    theta = tt;
    x_r = xxx;
    y_r = yyy;

    steer_cost = steer_c;
    steer = type;
    nEdges = global::settings.thetaStar.number_edges;
    READ_OR = 0;
    CHECK_SUCCESSOR = 0;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++) {
      costs[i] = c[i];
    }

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = orien[i];

    parent = new GNode_base();
    child = new GNode_base();

    goal_x = (int)global::settings.environment->goal().x;
    goal_y = (int)global::settings.environment->goal().y;
    start_x = (int)global::settings.environment->start().x;
    start_y = (int)global::settings.environment->start().y;
  }

  GNode(const GNode &n) {
    hasParent = n.hasParent;
    x = n.x;
    y = n.y;
    theta = n.theta;
    x_r = n.x_r;
    y_r = n.y_r;
    goal_x = n.goal_x;
    goal_y = n.goal_y;
    start_x = n.start_x;
    start_y = n.start_y;

    steer_cost = n.steer_cost;
    steer = n.steer;
    nEdges = n.nEdges;
    READ_OR = n.READ_OR;
    CHECK_SUCCESSOR = n.CHECK_SUCCESSOR;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++) {
      costs[i] = n.costs[i];
    }

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = n.orientations[i];

    parent = n.parent;
    child = n.child;
  }

  ~GNode() override {
    delete[] costs;
    delete[] orientations;
  }

  GNode &operator=(const GNode &n) {
    if (this == &n) return *this;

    hasParent = n.hasParent;
    x = n.x;
    y = n.y;
    theta = n.theta;
    x_r = n.x_r;
    y_r = n.y_r;
    goal_x = n.goal_x;
    goal_y = n.goal_y;
    start_x = n.start_x;
    start_y = n.start_y;

    steer_cost = n.steer_cost;
    steer = n.steer;
    nEdges = n.nEdges;
    READ_OR = n.READ_OR;
    CHECK_SUCCESSOR = n.CHECK_SUCCESSOR;

    orientations = new double[global::settings.thetaStar.number_edges.value()];
    costs = new double[global::settings.thetaStar.number_edges.value()];

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++) {
      costs[i] = 0;
    }

    for (int i = 0; i < global::settings.thetaStar.number_edges; i++)
      orientations[i] = 0;

    parent = n.parent;
    child = n.child;

    return *this;
  }

  bool operator==(const GNode &n) {
    return (x == n.x && y == n.y && theta == n.theta && x_r == n.x_r &&
            y_r == n.y_r);
  }

  bool operator!=(const GNode &n) {
    return (x != n.x || y != n.y || theta != n.theta || x_r != n.x_r ||
            y_r != n.y_r);
  }

  bool operator<(const GNode &n) const { return (steer_cost < n.steer_cost); }

  /// ============================================================================================
  /// getLineCost(), not _baseused
  /// ============================================================================================
  double *getLineCost() {
    double *local_cost;
    local_cost = new double[global::settings.thetaStar.number_edges.value()];
    local_cost[0] = 1;
    for (int i = 1; i < global::settings.thetaStar.number_edges; i++) {
      local_cost[i] = 1000;
    }

    return local_cost;
  }

  /// ============================================================================================
  /// setOrientation(GNode *parent_node)
  /// set the orientation of the current node as the one of the line connecting
  /// to the parent_node
  /// ============================================================================================
  bool setOrientation(GNode *parent_node) {
    double parent_x = parent_node->x;
    double parent_y = parent_node->y;
    double dt = .1;
    theta = atan2((y - parent_y) / dt, (x - parent_x) / dt);

    OMPL_INFORM("setOrientation %f instead of %f", theta, parent_node->theta);
    return true;
  }

  /// ============================================================================================
  /// bool IsSameState( GNode &rhs )
  /// Check if we have the same state
  /// ============================================================================================
  bool IsSameState(GNode &rhs) {
    return (this->x == rhs.x) && (this->y == rhs.y);
  }

  /// ============================================================================================
  /// void PrintNodeInfo()
  //  Print state of the Node, can be improved
  /// ============================================================================================
  void PrintNodeInfo() {
    char str[100];
    sprintf(str, "Node position : (%d,%d,%f)\n", x, y, theta);
    cout << str;
  }

  /// ============================================================================================
  /// GoalDistanceEstimate( GNode &nodeGoal )
  //  Here's the heuristic function that estimates the distance from a Node
  //  to the Goal.
  /// ============================================================================================
  float GoalDistanceEstimate(GNode &nodeGoal) {
    goal_x = nodeGoal.x;
    goal_y = nodeGoal.y;

    float xd = ((float)x - (float)nodeGoal.x);
    float yd = ((float)y - (float)nodeGoal.y);
    //    return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);

    return sqrt(xd * xd + yd * yd);
  }

  /// ============================================================================================
  ///  bool IsGoal( GNode &nodeGoal )
  /// Check if the node is the Goal
  /// ============================================================================================
  bool IsGoal(const GNode &nodeGoal) {
    goal_x = nodeGoal.x;
    goal_y = nodeGoal.y;

    if ((x == nodeGoal.x) && (y == nodeGoal.y)) {
      OMPL_DEBUG("Goal reached");
      return true;
    }

    return false;
  }

  /// ===========================================
  /// bool GetSuccessors( ThetaStarSearch<GNode> *thetastarsearch, GNode
  /// *parent_node ) This generates the successors to the given Node. It uses a
  /// helper function called AddSuccessor to give the successors to the
  /// Thetastar class. The Theta* specific initialisation is done for each node
  /// internally, so here you just set the state information that is specific to
  /// the application
  /// ============================================================================================
  bool GetSuccessors_or(ThetaStarSearch<GNode> *thetastarsearch,
                        GNode *parent_node) {
    int x = this->x;
    int y = this->y;
    double th = this->theta;
    int parent_x = -1;
    int parent_y = -1;

    if (parent_node) {
      parent_x = parent_node->x;
      parent_y = parent_node->y;
    } else {
      OMPL_ERROR("GNode has no parent node.");
      return false;
    }

    //
    // Try to find successors for each of the 8 directions (NW to SE)
    //

    // mod
    GNode NewNode;
    int cnt_successors = 0;

    auto *th1 = new GNode(x, y, th);
    auto *pr1 = new GNode(x - 1, y, 1.0 * M_PI);
    pr1->CHECK_SUCCESSOR = 1;
    th1->CHECK_SUCCESSOR = 1;
    if (lineofsight(th1, pr1) && !((parent_x == x - 1) && (parent_y == y))) {
      cnt_successors++;
      NewNode = GNode(x - 1, y, pr1->theta);
      NewNode.steer_cost = pr1->steer_cost;
      // NewNode.setParent(this);
      thetastarsearch->AddSuccessor(NewNode);
    }
    delete (th1);
    delete (pr1);

    auto *th2 = new GNode(x, y, th);
    auto *pr2 = new GNode(x, y - 1, 1.5 * M_PI);

    pr2->CHECK_SUCCESSOR = 1;
    th2->CHECK_SUCCESSOR = 1;
    if (lineofsight(th2, pr2) && !((parent_x == x) && (parent_y == y - 1))) {
      cnt_successors++;
      NewNode = GNode(x, y - 1, pr2->theta);
      NewNode.steer_cost = pr2->steer_cost;
      // NewNode.setParent(this);

      thetastarsearch->AddSuccessor(NewNode);
    }
    delete (th2);
    delete (pr2);

    auto *th3 = new GNode(x, y, th);
    auto *pr3 = new GNode(x + 1, y, 0.0 * M_PI);

    pr3->CHECK_SUCCESSOR = 1;
    th3->CHECK_SUCCESSOR = 1;
    if (lineofsight(th3, pr3) && !((parent_x == x + 1) && (parent_y == y))) {
      cnt_successors++;
      NewNode = GNode(x + 1, y, pr3->theta);
      NewNode.steer_cost = pr3->steer_cost;
      // NewNode.setParent(this);

      thetastarsearch->AddSuccessor(NewNode);
    }
    delete (th3);
    delete (pr3);

    auto *th4 = new GNode(x, y, th);
    auto *pr4 = new GNode(x + 1, y + 1, 0.25 * M_PI);

    pr4->CHECK_SUCCESSOR = 1;
    th4->CHECK_SUCCESSOR = 1;
    if (lineofsight(th4, pr4) &&
        !((parent_x == x + 1) && (parent_y == y + 1))) {
      cnt_successors++;
      NewNode = GNode(x + 1, y + 1, pr4->theta);
      NewNode.steer_cost = pr4->steer_cost;
      // NewNode.setParent(this);

      thetastarsearch->AddSuccessor(NewNode);
    }
    delete (th4);
    delete (pr4);

    auto *th5 = new GNode(x, y, th);
    auto *pr5 = new GNode(x - 1, y - 1, 1.25 * M_PI);

    pr5->CHECK_SUCCESSOR = 1;
    th5->CHECK_SUCCESSOR = 1;
    if (lineofsight(th5, pr5) &&
        !((parent_x == x - 1) && (parent_y == y - 1))) {
      cnt_successors++;

      NewNode = GNode(x - 1, y - 1, pr5->theta);
      NewNode.steer_cost = pr5->steer_cost;
      // NewNode.setParent(this);

      thetastarsearch->AddSuccessor(NewNode);
    }
    delete (th5);
    delete (pr5);

    auto *th6 = new GNode(x, y, th);
    auto *pr6 = new GNode(x + 1, y - 1, 1.75 * M_PI);

    pr6->CHECK_SUCCESSOR = 1;
    th6->CHECK_SUCCESSOR = 1;
    if (lineofsight(th6, pr6) &&
        !((parent_x == x + 1) && (parent_y == y - 1))) {
      cnt_successors++;

      NewNode = GNode(x + 1, y - 1, pr6->theta);
      NewNode.steer_cost = pr6->steer_cost;
      // NewNode.setParent(this);

      thetastarsearch->AddSuccessor(NewNode);
    }
    delete (th6);
    delete (pr6);

    auto *th7 = new GNode(x, y, th);
    auto *pr7 = new GNode(x - 1, y + 1, 0.75 * M_PI);

    pr7->CHECK_SUCCESSOR = 1;
    th7->CHECK_SUCCESSOR = 1;
    if (lineofsight(th7, pr7) &&
        !((parent_x == x - 1) && (parent_y == y + 1))) {
      cnt_successors++;

      NewNode = GNode(x - 1, y + 1, pr7->theta);
      NewNode.steer_cost = pr7->steer_cost;
      // NewNode.setParent(this);

      thetastarsearch->AddSuccessor(NewNode);
    }
    delete (th7);
    delete (pr7);

    auto *th8 = new GNode(x, y, th);
    auto *pr8 = new GNode(x, y + 1, 0.5 * M_PI);

    pr8->CHECK_SUCCESSOR = 1;
    th8->CHECK_SUCCESSOR = 1;
    if (lineofsight(th8, pr8) && !((parent_x == x) && (parent_y == y + 1))) {
      cnt_successors++;

      NewNode = GNode(x, y + 1, pr8->theta);
      NewNode.steer_cost = pr8->steer_cost;
      // NewNode.setParent(this);

      thetastarsearch->AddSuccessor(NewNode);
    }
    delete (th8);
    delete (pr8);

    if (cnt_successors < 1) {
      //      OMPL_DEBUG("GNode has no successors!");
      return false;
    } else {
      return true;
    }
  }

  /// ============================================================================================
  /// float GetCost( GNode &successor )
  /// Euclidean Distance Cost
  /// ============================================================================================
  double GetCost(GNode &successor) {
    double unit = global::settings.environment->unit();
    double dx = (successor.x - x) * unit;
    double dy = (successor.y - y) * unit;

    return std::sqrt(dx * dx + dy * dy);
  }

  // ============================================================================================
  /// float GetCostTraj( GNode &successor )
  /// get the cost of the trajectory that connect the current node to the
  /// successor
  /// ============================================================================================
  float GetCostTraj(GNode &successor) {
    // return successor.steer_cost;

    double unit = global::settings.environment->unit();
    return static_cast<float>(
        global::settings.ompl.objective
            ->motionCost(
                base::StateFromXY(x * unit, y * unit),
                base::StateFromXY(successor.x * unit, successor.y * unit))
            .value());

    //    double res = global::settings.steering->getBestCost(
    //        this->x, this->y, successor.x, successor.y);
    //    if (res < 0) {
    //      OMPL_ERROR(
    //          "Failing to compute the cost.. strange no lineofsight %d %d %d
    //          %d " "cost: %f, yaw: %f", this->x, this->y, successor.x,
    //          successor.y, res, global::settings.steering->getBestYaw(this->x,
    //          this->y, successor.x,
    //                                                successor.y));
    //      return 0;
    //    } else
    //      return res;
  }

  // ============================================================================================
  ///  float GetCostTraj( GNode &successor )
  /// get the cost of the trajectory that connect the current node to the
  /// successor
  /// ============================================================================================
  double GetCostTrajFromParent(GNode &parent, GNode &successor) {
    double unit = global::settings.environment->unit();
    return static_cast<double>(
        global::settings.ompl.objective
            ->motionCost(
                base::StateFromXY(parent.x * unit, parent.y * unit),
                base::StateFromXY(successor.x * unit, successor.y * unit))
            .value());
    //    double res = global::settings.steering->getBestCost(
    //        parent.x, parent.y, successor.x, successor.y);
    //
    //    if (res < 0) {
    //      OMPL_ERROR(
    //          "Failing to compute the cost.. strange no PARENTlineofsight %d
    //          %d %d "
    //          "%d cost: %f, yaw: %f",
    //          parent.x, parent.y, successor.x, successor.y, res,
    //          global::settings.steering->getBestYaw(parent.x, parent.y,
    //          successor.x,
    //                                                successor.y));
    //      return 0;
    //    } else
    //      return res;
  }

  /// ============================================================================================
  /// setType(int type)
  /// Set the Type of Node, if Type ==1, the node connects to the successor with
  /// a steer function
  /// ============================================================================================
  bool setType(int type) {
    this->steer = type;
    return true;
  }

  /// ===========================================
  /// bool GetSuccessors( ThetaStarSearch<GNode> *thetastarsearch, GNode
  /// *parent_node ) This generates the successors to the given Node. It uses a
  /// helper function called AddSuccessor to give the successors to the
  /// Thetastar class. The Theta* specific initialisation is done for each node
  /// internally, so here you just set the state information that is specific to
  /// the application
  /// ============================================================================================
  bool GetSuccessors(ThetaStarSearch<GNode> *thetastarsearch,
                     GNode *parent_node) {
    return GetSuccessors_or(thetastarsearch, parent_node);
  }

  /// ============================================================================================
  /// lineofsight(GNode *parent_node,GNode *successor)th
  /// It generates the trajectory based on the desired behaviour.
  /// if OPTIM_OR == 1, generates the best trajectory according to a cost and
  /// choose also the best orientation to use OTHERWISE, connect the two poses
  /// with the orientation being the one of the line connecting the two poses
  /// ============================================================================================
  bool lineofsight(GNode *parent_node, GNode *successor) {
    return line(parent_node, successor);
    //       return PlannerUtils::collides(
    //           Point(parent_node->x_r,
    //           parent_node->y_r).toState(parent_node->theta),
    //           Point(successor->x_r,
    //           successor->y_r).toState(successor->theta));
  }
};
