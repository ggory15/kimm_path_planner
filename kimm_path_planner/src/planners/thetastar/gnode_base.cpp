#include "kimm_path_planner/planners/thetastar/gnode_base.h"

#include "kimm_path_planner/base/PlannerSettings.h"
#include "kimm_path_planner/utils/PlannerUtils.hpp"

#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

bool GNode_base::isblock(double x, double y, double theta) {
  //    bool c = global::settings.environment->collides(x, y);
  //    QtVisualizer::drawNode(x, y, c ? Qt::red : Qt::darkGreen, 0.05);
  //    return c;
    double unit = global::settings.environment->unit();
  return !global::settings.environment->checkValidity(
      Point(x * unit, y * unit).toState(theta));

  // XXX ensure larger distance to prevent too low clearing distances
  return global::settings.environment->collides(x, y) ||
         global::settings.environment->bilinearDistance(x, y) <= 1.5;
}

 bool line(double x0, double y0, double y1, double x1) {
  double dx = (x1 - x0);
  double dy = (y1 - y0);
  const double theta = std::atan2(dy, dx);
  const double size = std::sqrt(dx * dx + dy * dy);
  const double scale = 0.05;
  dx = dx / size * scale;
  dy = dy / size * scale;

  const auto steps = (int)(size / std::sqrt(dx * dx + dy * dy));

  for (int j = 1; j < steps; ++j) {
    if (GNode_base::isblock(x0 + dx * j, y0 + dy * j, theta)) {
      return false;
    }
  }

  return true;
}

bool GNode_base::line(const GNode_base *parent_node,
                      const GNode_base *successor) {
//    double x0, y0, y1, x1;
//    x0 = parent_node->x;
//    y0 = parent_node->y;
//    x1 = successor->x;
//    y1 = successor->y;
//    bool c = ::line(x0, y0, y1, x1);

  bool c =
      !PlannerUtils::collides(parent_node->toState(), successor->toState());
//  OMPL_DEBUG("Line [%.2f %.2f %.2f] -- [%.2f %.2f %.2f] ?  %s",
//             parent_node->x_r, parent_node->y_r, parent_node->theta,
//             successor->x_r, successor->y_r, successor->theta,
//             (c ? "true" : "false"));
//      QtVisualizer::drawPath({Tpoint(x0, y0), Tpoint(x1, y1)}, !c ? Qt::red :
//      Qt::darkGreen);
  return c;
}
