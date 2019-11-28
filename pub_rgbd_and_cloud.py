#!/usr/bin/env python
# -*- coding: utf-8 -*-

import utils.lib_commons as lib_commons
from utils.lib_rgbd import MyCameraInfo, create_open3d_point_cloud_from_rgbd

from utils.lib_ros_rgbd_pub_and_sub import ColorImagePublisher, DepthImagePublisher, CameraInfoPublisher
from utils.lib_ros_point_cloud_pub_and_sub import PointCloudPublisher

import numpy as np
import cv2
import open3d
import argparse
import os
import sys
import rospy
ROOT = os.path.dirname(os.path.abspath(__file__))+'/'


def PAR(relative_path):  # Pre-Append Root to the relative path.
    if relative_path[0] == "/":  # This is absolute path.
        return relative_path
    return ROOT + relative_path


def parse_command_line_argumetns():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("-c", "--config_file", required=False,
                        default='config/rgbd_pub_config.yaml',
                        help="Path to the config file. "
                        "If the path is relative, this ROS package's path will be inserted to the beginning. "
                        "Default='config/rgbd_pub_config.yaml'")
    args = parser.parse_args(rospy.myargv()[1:])

    # -- Deal with relative path.
    rospy.loginfo("args.config_file:" + args.config_file)
    args.config_file = PAR(args.config_file)
    return args


def read_config_file(config_file_path):
    if not os.path.exists(config_file_path):
        raise RuntimeError("Config file doesn't exist: " + config_file_path)
    rospy.loginfo("Read config from: " + config_file_path)
    config = lib_commons.read_yaml_file(config_file_path)
    return config


class RgbdDataLoader(object):
    ''' Data loader for color, depth, camera_info, and point cloud. '''

    def __init__(self, config):
        '''
        Arguments:
            config {dict}: the output from `read_config_file()`.
        '''
        self._config = config
        self.reset()

    def read_next_data(self):
        if not self.has_data():
            raise RuntimeError("The data is empty. Cannot read next data")

        # Initalize empty data.
        color, depth, camera_info, cloud = None, None, None, None

        # Read color image.
        if self._is_read_color:
            filename = self._color_names[self._cnt_images]
            color = cv2.imread(filename, cv2.IMREAD_COLOR)
            assert(color is not None)

        # Read depth image.
        if self._is_read_depth:
            filename = self._depth_names[self._cnt_images]
            depth = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
            assert(depth is not None)

        # Read camera_info (which has already been read).
        if self._is_read_camera_info:
            camera_info = self._camera_info

        # Create point cloud.
        if self._is_pub_cloud:
            cloud = create_open3d_point_cloud_from_rgbd(
                color, depth,
                camera_info,
                self._depth_unit,
                self._depth_trunc)

        # -- Return
        self._cnt_images += 1
        return color, depth, camera_info, cloud

    def has_data(self):
        # False if: Image counter >= total color images.
        if self._color_names and self._cnt_images >= len(self._color_names):
            return False
        # False if: Image counter >= total depth images.
        if self._depth_names and self._cnt_images >= len(self._depth_names):
            return False
        return True

    def reset(self):
        config = self._config

        # -- Read settings from the config file.
        self._is_pub_color = config["color"]["is_publish"]
        self._is_pub_depth = config["depth"]["is_publish"]
        self._is_pub_camera_info = config["camera_info"]["is_publish"]
        self._is_pub_cloud = config["point_cloud"]["is_publish"]
        self._depth_unit = float(config["depth"]["depth_unit"])
        self._depth_trunc = float(config["depth"]["depth_trunc"])

        color_folder = PAR(config["color"]["folder"])
        depth_folder = PAR(config["depth"]["folder"])
        camera_info_file_path = PAR(config["camera_info"]["file"])

        # -- Whether read data or not.
        # If we need to publish data, then we need to read it first.
        # Or, if we need point cloud, we also need to read color/depth/camera_info.
        self._is_read_color = self._is_pub_color or self._is_pub_cloud
        self._is_read_depth = self._is_pub_depth or self._is_pub_cloud
        self._is_read_camera_info = self._is_pub_camera_info or self._is_pub_cloud

        # -- Read color/depth image filenames
        color_names, depth_names = None, None

        # color only.
        if (self._is_read_color) and (not self._is_read_depth):
            color_names = lib_commons.get_filenames(color_folder)

        # depth only.
        elif (not self._is_read_color) and (self._is_read_depth):
            depth_names = lib_commons.get_filenames(depth_folder)

        # color and depth.
        elif (self._is_read_color) and (self._is_read_depth):
            # If both `color` and `depth`,
            # then their filenames are supposed to be the same.
            color_base_names = lib_commons.get_filenames(
                color_folder, is_base_name=True)
            color_names = [color_folder + s for s in color_base_names]
            depth_names = [depth_folder + s for s in color_base_names]

        # store result
        self._color_names = color_names
        self._depth_names = depth_names

        # -- Read camera info.
        if self._is_read_camera_info:
            self._camera_info = MyCameraInfo(camera_info_file_path)
        else:
            self._camera_info = None

        # -- Variables
        self._cnt_images = 0  # Number of images read from the folder.

    def is_pub_color(self):
        return self._is_pub_color

    def is_pub_depth(self):
        return self._is_pub_depth

    def is_pub_camera_info(self):
        return self._is_pub_camera_info

    def is_pub_cloud(self):
        return self._is_pub_cloud


def main(config):
    ''' 
    Arguments:
        config {dict}: Read from a config file such as `config/run_publisher.launch`. 
    '''

    # -- Set data loader
    rgbd_data = RgbdDataLoader(config)

    # -- Set publishers.
    ns = config["ros_topic_namespace"] + "/"
    frame_id = config["frame_id"]

    topic_name = ns + config["color"]["ros_topic"]
    pub_color = ColorImagePublisher(topic_name)

    topic_name = ns + config["depth"]["ros_topic"]
    pub_depth = DepthImagePublisher(topic_name)

    topic_name = ns + config["camera_info"]["ros_topic"]
    pub_camera_info = CameraInfoPublisher(topic_name)

    topic_name = ns + config["point_cloud"]["ros_topic"]
    pub_cloud = PointCloudPublisher(topic_name)

    # -- Loop and publish.
    is_loop_forever = config["is_loop_forever"]
    loop_rate = rospy.Rate(config["pub_frequency"])

    while not rospy.is_shutdown():

        # -- Read next data.
        if not rgbd_data.has_data():
            if is_loop_forever:  # Reset data loader.
                rgbd_data.reset()
            else:  # Stop program.
                break
        color, depth, camera_info, cloud = rgbd_data.read_next_data()

        # -- Publish data.
        rospy.loginfo("=================================================")

        # Color.
        if rgbd_data.is_pub_color():
            pub_color.publish(color, frame_id)
            rospy.loginfo("Publish color image, "
                          "shape = {}.".format(color.shape))

        # Depth.
        if rgbd_data.is_pub_depth():
            pub_depth.publish(depth, frame_id)
            rospy.loginfo("Publish depth image, "
                          "shape = {}.".format(depth.shape))

        # Camera_info.
        if rgbd_data.is_pub_camera_info():
            pub_camera_info.publish(
                camera_info.width(),
                camera_info.height(),
                camera_info.intrinsic_matrix(),
                frame_id)
            rospy.loginfo("Publish camera info.")

        # Point_cloud.
        if rgbd_data.is_pub_cloud():
            pub_cloud.publish(cloud,
                              cloud_format="open3d",
                              frame_id=frame_id)

            num_points = np.asarray(cloud.points).shape[0]
            rospy.loginfo("Publish point cloud, "
                          "{} points.".format(num_points))

        # -- Wait for publish.
        loop_rate.sleep()


if __name__ == '__main__':
    node_name = "pub_rgbd_and_cloud"
    rospy.init_node(node_name)
    args = parse_command_line_argumetns()
    config = read_config_file(args.config_file)
    main(config)
    rospy.logwarn("Node `{}` stops.".format(node_name))
