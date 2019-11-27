#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This script gives exampe code for subscribing:
    (1) Color image.
    (2) Depth image.
    (3) Camera info.
    (4) Point cloud (subscribed as open3d format).
'''

from utils.lib_ros_rgbd_pub_and_sub import ColorImageSubscriber, DepthImageSubscriber, CameraInfoSubscriber
from utils.lib_ros_point_cloud_pub_and_sub import PointCloudSubscriber

from pub_rgbd_and_cloud import PAR, parse_command_line_argumetns, read_config_file

import numpy as np
import rospy
import argparse


def main(config):
    ''' 
    Arguments:
        config {dict}: Read from a config file such as `config/run_publisher.launch`. 
    '''

    # -- Set subscribers.
    ns = config["ros_topic_namespace"] + "/"
    frame_id = config["frame_id"]

    topic_name = ns + config["color"]["ros_topic"]
    sub_color = ColorImageSubscriber(topic_name)

    topic_name = ns + config["depth"]["ros_topic"]
    sub_depth = DepthImageSubscriber(topic_name)

    topic_name = ns + config["camera_info"]["ros_topic"]
    sub_camera_info = CameraInfoSubscriber(topic_name)

    topic_name = ns + config["point_cloud"]["ros_topic"]
    sub_cloud = PointCloudSubscriber(topic_name)

    cnt_1, cnt_2, cnt_3, cnt_4 = 0, 0, 0, 0  # color, depth, camera_info, cloud

    # -- Loop and subscribe.
    while not rospy.is_shutdown():

        # Color.
        if sub_color.has_image():
            color = sub_color.get_image()
            cnt_1 += 1
            rospy.loginfo("Subscribe {}: color image, "
                          "shape={}".format(
                              cnt_1, color.shape))

        # Depth.
        if sub_depth.has_image():
            depth = sub_depth.get_image()
            cnt_2 += 1
            rospy.loginfo("Subscribe {}: depth image, "
                          "shape={}".format(
                              cnt_2, depth.shape))

        # Camera_info.
        if sub_camera_info.has_camera_info():
            camera_info = sub_camera_info.get_camera_info()
            cnt_3 += 1
            rospy.loginfo("Subscribe {}: camera_info, "
                          "fx={}, fy={}.".format(
                              cnt_3,
                              camera_info.K[0],
                              camera_info.K[4],
                          ))

        # Point_cloud.
        if sub_cloud.has_cloud():
            open3d_cloud = sub_cloud.get_cloud()
            cnt_4 += 1
            num_points = np.asarray(open3d_cloud.points).shape[0]
            rospy.loginfo("Subscribe {}: point cloud, "
                          "{} points.".format(
                              cnt_4, num_points))

        rospy.sleep(0.1)


if __name__ == '__main__':
    node_name = "sub_rgbd_and_cloud"
    rospy.init_node(node_name)
    args = parse_command_line_argumetns()
    config = read_config_file(args.config_file)
    main(config)
    rospy.logwarn("Node `{}` stops.".format(node_name))
