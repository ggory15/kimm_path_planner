#include "ros/ros.h"
#include "kimm_path_planner_ros_interface/plan_mobile_path.h"
#include "kimm_path_planner_ros_interface/action_mobile_path.h"

#include "kimm_path_planner_ros_interface/MobileTrajectory.h"
#include "std_msgs/Bool.h"

#include <yaml-cpp/yaml.h>

#include <kimm_path_planner/base/GridMaze.h>
#include <kimm_path_planner/utils/PathEvaluation.hpp>


int main(int argc, char **argv)
{
   ros::init(argc, argv, "MobilePathPlannerClient");
   ros::NodeHandle nh("~");

   ros::ServiceClient ros_mobile_planner_client = nh.serviceClient<kimm_path_planner_ros_interface::plan_mobile_path>("/ns0/kimm_path_planner_ros_interface_server/plan_mobile_path");
   kimm_path_planner_ros_interface::plan_mobile_path mobile_srv; 

   geometry_msgs::Pose2D target_pose_service;
   geometry_msgs::Pose2D current_pose_service;

   // Current pose
   current_pose_service.x = 0.0;
   current_pose_service.y = 0.0;
   current_pose_service.theta = 0.0;

   // Target Pose
   target_pose_service.x = 2.0;
   target_pose_service.y = 2.0;
   target_pose_service.theta = 0.0;
   //obs_xyt = [(2.7251, -0.5276, 0.6), (4.5502, 2.0, 0.6), (4.5, -2.825, 0.1)]

   // Obstacle
   kimm_path_planner_ros_interface::Obstacle2D Obstacle1;
   Obstacle1.x1.data = 2.7251;
   Obstacle1.y1.data = -0.5276;
   Obstacle1.x2.data = 3.7251;
   Obstacle1.y2.data = -1.5276;

   // Get current state & target pose
   mobile_srv.request.current_mobile_state = current_pose_service;
   mobile_srv.request.target_mobile_pose = target_pose_service;
   mobile_srv.request.Obstacles2D.push_back(Obstacle1);



   if (ros_mobile_planner_client.call(mobile_srv)) // call the service and return the response value
   {
      //ROS_INFO("send srv, srv.Request.a and b : %1d, %1d", (long int)srv.request.a, (long int)srv.request.b);
      //ROS_INFO("recieve srv, srv.Response.result : %1d", (long int)srv.response.result);
   }
   else
   {
      ROS_ERROR("Failed to call service ros_tutorial_srv");
      return 1;
   }

   return 0;
}
