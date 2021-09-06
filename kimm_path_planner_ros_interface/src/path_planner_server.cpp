#include "ros/ros.h"
#include "kimm_path_planner_ros_interface/plan_mobile_path.h"
#include "kimm_path_planner_ros_interface/action_mobile_path.h"

#include "kimm_path_planner_ros_interface/MobileTrajectory.h"
#include "std_msgs/Bool.h"

#include <yaml-cpp/yaml.h>

#include <kimm_path_planner/base/GridMaze.h>
#include <kimm_path_planner/utils/PathEvaluation.hpp>
#include <kimm_path_planner/planners/thetastar/ThetaStar.h>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

using json = nlohmann::json;

#define TIMEOPT 

#ifdef TIMEOPT
    #include <kimm_trajectory_smoother/Trajectory.h>
    #include <kimm_trajectory_smoother/Path.h>
    kimmtraj::Trajectory *trajectory_generator_;
    kimmtraj::stdlist_Eigenvec wayPoints_;
    int playTime_;
#endif

bool is_calc_, is_run_;

ros::ServiceServer planning_server_;
ros::ServiceServer action_server_;
std::shared_ptr<GridMaze> environment_;
std::string save_location_, robot_;

kimm_path_planner_ros_interface::MobileTrajectory output_trajectory_;
Eigen::Vector2d offset_; 
bool calculation(kimm_path_planner_ros_interface::plan_mobile_path::Request &req, kimm_path_planner_ros_interface::plan_mobile_path::Response &res)
{
    double scaling = 0.1;
    environment_ = GridMaze::createSimple(100, 100);
    environment_->setStart(Point(req.current_mobile_state.x / scaling + offset_(0), req.current_mobile_state.y / scaling + offset_(1) ));
    environment_->setGoal(Point(req.target_mobile_pose.x / scaling + offset_(0), req.target_mobile_pose.y / scaling + offset_(1)) );
    environment_->setThetas(req.current_mobile_state.theta, req.target_mobile_pose.theta);

    for (int i=0; i< req.Obstacles2D.size(); i++){
        double x_center = (req.Obstacles2D[i].x1.data + req.Obstacles2D[i].x2.data) / 2.0 / scaling;
        double y_center = (req.Obstacles2D[i].y1.data + req.Obstacles2D[i].y2.data) / 2.0 / scaling;
        double x_width =  abs(req.Obstacles2D[i].x1.data - req.Obstacles2D[i].x2.data) / 2.0 / scaling + 0.0 / scaling;
        double y_width =  abs(req.Obstacles2D[i].y1.data - req.Obstacles2D[i].y2.data) / 2.0 / scaling + 0.0 / scaling;
        environment_->fill(Rectangle(x_center - x_width + offset_(0),  y_center - y_width  + offset_(1), x_center + x_width  + offset_(0), y_center + y_width + offset_(1)), true);
    }

    global::settings.environment = environment_;
    global::settings.steer.steering_type = Steering::STEER_TYPE_DUBINS;
    //global::settings.steer.steering_type = Steering::STEER_TYPE_REEDS_SHEPP;
	global::settings.steer.car_turning_radius = 0.3 / scaling;
    global::settings.benchmark.smoothing.ompl_simplify_max = true;
	global::settings.steer.initializeSteering();
    global::settings.env.collision.collision_model = robot::ROBOT_POINT;
    global::settings.max_planning_time = 0.8;    
    //global::settings.env.collision.robot_shape_source = robot_;
	global::settings.env.collision.initializeCollisionModel();
    //global::settings.ompl.optimization_objective.value() = "min_pathlength";

#ifdef TIMEOPT
    wayPoints_.clear();
    playTime_ = 0;
#endif

    Log::instantiateRun();
    auto info = nlohmann::json({{"plans", {}}});
    is_calc_= PathEvaluation::evaluateSmoothers<InformedRRTstarPlanner>(info);     
    is_run_ = false;

    if (is_calc_){
#ifdef TIMEOPT
        Log::log(info);
        Log::save(save_location_);
        
        std::ifstream i(save_location_);
        json j_tmp;
        i >> j_tmp;
        auto v2 =  j_tmp["runs"][0]["plans"]["InformedRRTstar"]["smoothing"]["ompl_simplify_max"]["trajectory"].get<std::vector<std::vector<double>>>();
       
        geometry_msgs::Pose2D pose;

        for (int i=0; i<v2.size(); i++){
            while (v2[i][2] < 0.){
				v2[i][2] += 2*M_PI;
			}
            if (i > 0){
                while (v2[i][2] - v2[i-1][2]< -M_PI){
				    v2[i][2] += 2*M_PI;
                }
                while (v2[i][2] - v2[i-1][2] > M_PI){
                    v2[i][2] -= 2*M_PI;
                }
            }
            wayPoints_.extend(Eigen::Vector3d( (v2[i][0] - offset_(0)) * scaling, (v2[i][1]  - offset_(1)) * scaling, v2[i][2]) );
        }

        double dt = 0.1;
        trajectory_generator_ = new kimmtraj::Trajectory(kimmtraj::Path(wayPoints_, dt), Eigen::Vector3d(0.85, 0.85, 0.85), Eigen::Vector3d(1.0, 1.0, 1.0));
        double duration = trajectory_generator_->getDuration();

        while (playTime_ * dt < duration)
		{
			
			pose.x = trajectory_generator_->getPosition(playTime_ * dt)[0];
			pose.y = trajectory_generator_->getPosition(playTime_ * dt)[1];
			pose.theta = trajectory_generator_->getPosition(playTime_ * dt)[2];

			res.mobile_path.points.push_back(pose);
			
			playTime_ ++; 
		}

        output_trajectory_ = res.mobile_path;
        i.close();
        
#else
        Log::log(info);
        Log::save(save_location_);
        
        std::ifstream i(save_location_);
        json j_tmp;
        i >> j_tmp;
        auto v2 =  j_tmp["runs"][0]["plans"]["BFMT"]["smoothing"]["ompl_simplify_max"]["trajectory"].get<std::vector<std::vector<double>>>();
       
        geometry_msgs::Pose2D pose;

        for (int i=0; i<v2.size(); i++){
            pose.x = v2[i][0] - offset_(0);
            pose.y = v2[i][1] - offset_(1);
            pose.theta = v2[i][2];
            
            res.mobile_path.points.push_back(pose);
        }

        output_trajectory_ = res.mobile_path;
        i.close();
#endif
        return true;
    }
    else{
    	res.mobile_path.points.clear();
    	output_trajectory_.points.clear();
        return true;
    }
}
bool updateTrajectory(kimm_path_planner_ros_interface::action_mobile_path::Request &req, kimm_path_planner_ros_interface::action_mobile_path::Response &res)
{
    if (is_calc_ && !is_run_){
    	is_run_ = true;
 	    ROS_INFO("is_run: [%d]", is_run_ ); // res.q_trajectory.joint_names[0].c_str()
        res.mobile_path = output_trajectory_;
        return true;
    }
    else{
    	output_trajectory_.points.clear();
    	res.mobile_path.points.clear();
        return true;
    }
}

int main(int argc, char **argv)
{

    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    srand(spec.tv_nsec);

    ros::init(argc, argv, "MobilePathPlannerServer");
    ros::NodeHandle nh("~");

    nh.getParam("save_path", save_location_);    
    nh.getParam("robot_planning_model", robot_);
    
    // Path Planning Server
    offset_(0) = 50.0;
    offset_(1) = 50.0;  
    
    
    planning_server_ = nh.advertiseService("plan_mobile_path", calculation);
    action_server_ = nh.advertiseService("action_mobile_path", updateTrajectory);
    
    
    ROS_INFO("ready srv server!");
    
    while (ros::ok())
    { 


        ros::spinOnce();       
    } 

    return 0;
}
