#include <iostream>

#include <gtest/gtest.h>

#include "kimm_path_planner/base/GridMaze.h"
#include "kimm_path_planner/utils/PathEvaluation.hpp"
#include "kimm_path_planner/planners/thetastar/ThetaStar.h"

#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

TEST(TestSuite, testCase1){

	auto environment = GridMaze::createRandomCorridor(150, 50, 3, 30, 1);
	//cout << *GridMaze::createSimple() << endl;
	global::settings.environment = environment;
	global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
	global::settings.steer.car_turning_radius = 3;
	global::settings.steer.initializeSteering();
	global::settings.env.collision.collision_model = robot::ROBOT_POINT;
	//global::settings.env.collision.robot_shape_source = "/home/ggory15/kimm_catkin/src/kimm_path_planner/kimm_path_planner/robot_model/car.svg";
	//global::settings.env.collision.initializeCollisionModel();

	auto info = nlohmann::json({{"plans", {}}});
	
	PathEvaluation::evaluate<BFMTPlanner>(info);
	Log::log(info);
	Log::save("smoothing_cc_dubins.json");

	std::ifstream i("smoothing_cc_dubins.json");
	json j_tmp;
	i >> j_tmp;
	double v1;

	auto v2 =  j_tmp["runs"][0]["plans"]["BFMT"]["path"].get<std::vector<std::vector<double>>>();
	cout << "v1: " << v2[0] << endl;
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}