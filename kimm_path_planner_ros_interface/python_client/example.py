#!/usr/bin/env python
# -*- coding:utf-8 -*-

import random
import math
import rospy
import tf
from tf.transformations import quaternion_from_euler

import std_msgs.msg
import geometry_msgs.msg
import kimm_path_planner_ros_interface.msg
import kimm_path_planner_ros_interface.srv
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

RED = std_msgs.msg.ColorRGBA(1.0, 0.0, 0.0, 1.0)
GREEN = std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, 1.0)
WHITE = std_msgs.msg.ColorRGBA(1.0, 1.0, 1.0, 1.0)
t_BLUE = std_msgs.msg.ColorRGBA(0.0, 0.0, 1.0, 0.5)

DEFAULT_QUAT = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)


def _create_marker_init():
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time()
    marker.ns = "my_namespace"
    marker.id = random.randint(0, 2048)
    return marker


def create_marker(position, orientation, scale, color_rgba, shape):
    # http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
    marker = _create_marker_init()
    marker.type = shape
    marker.action = marker.ADD
    marker.color = color_rgba
    marker.pose.position = position
    marker.pose.orientation = orientation
    marker.scale = scale
    return marker


def create_line(point_list, color_rgba):
    marker = _create_marker_init()
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.color = color_rgba
    marker.pose.position = geometry_msgs.msg.Point(0, 0, 0)
    marker.pose.orientation = DEFAULT_QUAT
    marker.points = [geometry_msgs.msg.Point(p[0], p[1], p[2]) for p in point_list]
    marker.scale = geometry_msgs.msg.Vector3(0.005, 0, 0)
    return marker


def joint_trajectory_deg2rad(joint_trajectory):
    for p in joint_trajectory.points:
        p.positions = [math.radians(t) for t in p.positions]
    return joint_trajectory


class Example:
    def __init__(self):
        self.listener = tf.TransformListener()
        rospy.wait_for_service("ns0/kimm_path_planner_ros_interface_server/plan_mobile_path")
        self.plan_mobile_motion = rospy.ServiceProxy(
            "ns0/kimm_path_planner_ros_interface_server/plan_mobile_path",
            kimm_path_planner_ros_interface.srv.plan_mobile_path,
        )
        self.req = kimm_path_planner_ros_interface.srv.plan_mobile_pathRequest()
        self.resp = kimm_path_planner_ros_interface.srv.plan_mobile_pathResponse()

    def test(self):
        self.req.current_mobile_state = geometry_msgs.msg.Pose2D(2.0, 0.0, 0)
        self.req.target_mobile_pose = geometry_msgs.msg.Pose2D(0.6, 0, math.pi)
        obs_xyt = [(-0.5, -0.5, 0.5, 0.5), (-1.4, 1.6, -0.6, 2.4), (0.7, 2.7, 1.3, 3.3)]
        self.req.Obstacles2D = [
            kimm_path_planner_ros_interface.msg.Obstacle2D(
                x1=std_msgs.msg.Float64(x1),
                y1=std_msgs.msg.Float64(y1),
                x2=std_msgs.msg.Float64(x2),
                y2=std_msgs.msg.Float64(y2),
            ) for x1, y1, x2, y2 in obs_xyt
        ]

        try:
            self.resp = self.plan_mobile_motion(self.req)
            print(self.resp.mobile_path.points)
            rospy.logwarn("========== Done! ==========")
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def srv_visualize(self):
        arrow_scale = geometry_msgs.msg.Vector3(0.1, 0.015, 0.015)
        req_pub = rospy.Publisher("kimm_mobile_plan_markers/mobile/request", MarkerArray, queue_size=1, latch=True)
        resp_pub = rospy.Publisher("kimm_mobile_plan_markers/mobile/response", MarkerArray, queue_size=1, latch=True)
        req_markers = MarkerArray()
        resp_markers = MarkerArray()

        # Request ==============
        # Start
        current_Pose2D = self.req.current_mobile_state
        quat = quaternion_from_euler(0.0, 0.0, -current_Pose2D.theta)
        req_markers.markers.append(
            create_marker(
                geometry_msgs.msg.Point(current_Pose2D.x, current_Pose2D.y, 0.0),
                geometry_msgs.msg.Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
                arrow_scale,
                GREEN,
                Marker.ARROW,
            ))
        # Goal
        target_Pose2D = self.req.target_mobile_pose
        quat = quaternion_from_euler(0.0, 0.0, target_Pose2D.theta)
        req_markers.markers.append(
            create_marker(
                geometry_msgs.msg.Point(target_Pose2D.x, target_Pose2D.y, 0.0),
                geometry_msgs.msg.Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
                arrow_scale,
                RED,
                Marker.ARROW,
            ))
        # Obstacles
        for obs in self.req.Obstacles2D:
            req_markers.markers.append(
                create_marker(
                    geometry_msgs.msg.Point( (obs.x1.data + obs.x2.data)/2.0 , (obs.y1.data + obs.y2.data)/2.0, 0.0),
                    geometry_msgs.msg.Quaternion(0, 0, 0, 1),
                    geometry_msgs.msg.Vector3(np.abs((obs.x1.data - obs.x2.data)), np.abs((obs.y1.data - obs.y2.data)), 0.1),
                    t_BLUE,
                    Marker.CUBE,
                ))
        # Response ==============
        for pose in self.resp.mobile_path.points:
            quat = quaternion_from_euler(0.0, 0.0, pose.theta)
            resp_markers.markers.append(
                create_marker(
                    geometry_msgs.msg.Point(pose.x, pose.y, 0.0),
                    geometry_msgs.msg.Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
                    arrow_scale,
                    WHITE,
                    Marker.ARROW,
                ))

        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            req_pub.publish(req_markers)
            resp_pub.publish(resp_markers)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("Example")
    ex = Example()
    ex.test()
    ex.srv_visualize()
    rospy.spin()
