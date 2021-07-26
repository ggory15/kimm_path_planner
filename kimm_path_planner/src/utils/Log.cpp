#include "kimm_path_planner/utils/Log.h"

#include <stdio.h>
#include <stdlib.h>

#include "kimm_path_planner/smoothers/grips/GRIPS.h"

nlohmann::json Log::_json = {{"runs", nlohmann::json::array()}};
nlohmann::json Log::_currentRun;

void Log::instantiateRun() {
  auto time =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char mbstr[100];
  std::strftime(mbstr, sizeof(mbstr), "%F %T", std::localtime(&time));
  std::string tstr(mbstr);
  tstr = tstr.substr(0, tstr.length() - 1);
  _currentRun = {{"globals",
                  {
                      {"time", tstr},
                  }},
                 {"runs", nlohmann::json::array()}};

  _currentRun["settings"] = nlohmann::json(global::settings)["settings"];
  _currentRun["settings"]["steering"] =
      Steering::to_string(global::settings.steer.steering_type);
}

void Log::log(const PathStatistics &stats) {
  nlohmann::json runStats = stats;
  runStats["ps_roundStats"] = GRIPS::statsPerRound;
  _currentRun["runs"].push_back(runStats);
}

void Log::log(const nlohmann::json &stats) {
  _currentRun["runs"].push_back(stats);
}

void Log::save(std::string filename, const std::string &path) {
  if (filename.empty()) filename = Log::filename() + (std::string) ".json";
  std::ofstream o(filename);
  o << std::setw(4) << _currentRun << std::endl;
  o.close();
  OMPL_INFORM("Saved log at %s", filename.c_str());
}

void Log::storeRun() { _json["runs"].push_back(_currentRun); }

bool replace(std::string &str, const std::string &from, const std::string &to) {
  // thanks to https://stackoverflow.com/a/3418285
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos) return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

std::string Log::filename() {
  auto env_name = global::settings.environment->name();
  replace(env_name, "/", "_");
  replace(env_name, ".", "_");
  replace(env_name, ":", "_");
  replace(env_name, "*", "_");
  return _currentRun["settings"]["steering"].get<std::string>() + " " +
         std::to_string(global::settings.environment->width()) + "x" +
         std::to_string(global::settings.environment->height()) + " " +
         env_name + " " + _currentRun["globals"]["time"].get<std::string>();
}

std::vector<std::array<double, 2>> Log::serializePath(
    const std::vector<Point> &path) {
  std::vector<std::array<double, 2>> r;
  for (auto &p : path) r.push_back({p.x, p.y});
  return r;
}

std::vector<std::array<double, 3>> Log::serializeTrajectory(
    const ompl::geometric::PathGeometric &t, bool interpolate) {
  ompl::geometric::PathGeometric traj = t;

  if (interpolate) traj = PlannerUtils::interpolated(t);
  std::vector<std::array<double, 3>> r;

  double x, y, yaw;

  for (auto i = 0u; i < traj.getStateCount(); ++i) {
    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK) {
      const auto *compState =
          traj.getState(i)->as<ob::CompoundStateSpace::StateType>();
      const auto *se2state = compState->as<ob::SE2StateSpace::StateType>(0);
      x = se2state->getX();
      y = se2state->getY();
      yaw = se2state->getYaw();
    } else {
      const auto *s = traj.getState(i)->as<State>();
      x = s->getX();
      y = s->getY();
      yaw = s->getYaw();
    }
    r.push_back({x, y, yaw});
  }
  return r;
}

std::vector<std::array<double, 3>> Log::serializeTrajectory(
    const ompl::control::PathControl &t, bool interpolate) {
  ompl::control::PathControl traj = t;
  if (interpolate) traj = PlannerUtils::interpolated(t);
  std::vector<std::array<double, 3>> r;
  for (auto i = 0u; i < traj.getStateCount(); ++i) {
    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_CAR) {
      const auto *s = traj.getState(i)->as<State>();
      r.push_back({s->getX(), s->getY(), s->getYaw()});
    }

    if (global::settings.forwardpropagation.forward_propagation_type ==
        ForwardPropagation::FORWARD_PROPAGATION_TYPE_KINEMATIC_SINGLE_TRACK) {
      const auto *s = traj.getState(i)->as<ob::CompoundStateSpace::StateType>();
      const auto *se2state = s->as<ob::SE2StateSpace::StateType>(0);
      r.push_back({se2state->getX(), se2state->getY(), se2state->getYaw()});
    }
  }
  return r;
}