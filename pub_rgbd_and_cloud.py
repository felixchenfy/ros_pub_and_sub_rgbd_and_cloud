#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils.lib_ros_rgbd_pub_and_sub import ColorImagePublisher, DepthImagePublisher, CameraInfoPublisher

import numpy as np
import cv2
import yaml
import numpy as np
import argparse
import os
import sys
import simplejson
import glob
import rospy
ROOT = os.path.dirname(os.path.abspath(__file__))+'/'


def parse_command_line_argumetns():
    parser = argparse.ArgumentParser(
        description="Publish rgbd images and/or point cloud to ROS topics.")
    parser.add_argument("-c", "--config_file", required=False,
                        default='config/publisher_settings.yaml',
                        help="Path to the config file. "
                        "If the path is relative, this ROS package's path will be inserted to the beginning. "
                        "Default='config/publisher_settings.yaml'")
    args = parser.parse_args()
    if args.config_file[0] != "/":  # relative path --> absolute path
        args.config_file = ROOT + args.config_file
    return args


def read_yaml_file(file_path):
    with open(file_path, 'r') as stream:
        data = yaml.safe_load(stream)
    return data


def read_json_file(file_path):
    with open(file_path, 'r') as f:
        data = simplejson.load(f)
    return data


def read_config_file(config_file_path):
    if not os.path.exists(config_file_path):
        raise RuntimeError("Config file doesn't exist: " + config_file_path)
    rospy.loginfo("Read config from: " + config_file_path)
    config = read_yaml_file(config_file_path)
    return config


class MyCameraInfo():
    def __init__(self, camera_info_json_file_path):
        data = read_json_file(camera_info_json_file_path)
        self._width = data["width"]  # float
        self._height = data["height"]  # float
        self._intrinsic_matrix = data["intrinsic_matrix"]  # list of float

    def width(self):
        return self._width

    def height(self):
        return self._height

    def intrinsic_matrix(self):
        return self._intrinsic_matrix


def get_filenames(folder, is_base_name=False):
    ''' Get all filenames under the specific folder. 
    e.g.:
        full name: data/rgb/000001.png
        base name: 000001.png 
    '''
    full_names = sorted(glob.glob(folder + "/*"))
    if is_base_name:
        base_names = [name.split("/")[-1] for name in full_names]
        return base_names
    else:
        return full_names


def create_point_cloud_from_rgbd(rgb, depth, camera_info):
    cloud = None
    return cloud


class RgbdDataLoader(object):
    ''' Data loader for rgb, depth, camera_info, and point cloud. '''

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
        rgb, depth, camera_info, cloud = None, None, None, None

        # Read rgb image.
        if self._is_read_rgb:
            filename = self._rgb_names[self._cnt_images]
            rgb = cv2.imread(filename, cv2.IMREAD_COLOR)
            assert(rgb is not None)

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
            cloud = create_point_cloud_from_rgbd(rgb, depth, camera_info)

        # -- Return
        self._cnt_images += 1
        return rgb, depth, camera_info, cloud

    def has_data(self):
        # False if: Image counter >= total rgb images.
        if self._rgb_names and self._cnt_images >= len(self._rgb_names):
            return False
        # False if: Image counter >= total depth images.
        if self._depth_names and self._cnt_images >= len(self._depth_names):
            return False
        return True

    def reset(self):
        config = self._config

        # -- Read settings from the config file.
        self._is_pub_rgb = config["rgb"]["is_publish"]
        self._is_pub_depth = config["depth"]["is_publish"]
        self._is_pub_camera_info = config["camera_info"]["is_publish"]
        self._is_pub_cloud = config["point_cloud"]["is_publish"]
        rgb_folder = config["rgb"]["folder"]
        depth_folder = config["depth"]["folder"]
        camera_info_file_path = config["camera_info"]["file"]

        # -- Whether read data or not.
        # If we need to publish data, then we need to read it first.
        # Or, if we need point cloud, we also need to read rgb/depth/camera_info.
        self._is_read_rgb = self._is_pub_rgb or self._is_pub_cloud
        self._is_read_depth = self._is_pub_depth or self._is_pub_cloud
        self._is_read_camera_info = self._is_pub_camera_info or self._is_pub_cloud

        # -- Read rgb/depth image filenames
        rgb_names, depth_names = None, None
        if (self._is_read_rgb) and (not self._is_read_depth):  # rgb only.
            rgb_names = get_filenames(rgb_folder)
        elif (not self._is_read_rgb) and (self._is_read_depth):  # depth only.
            depth_names = get_filenames(depth_folder)
        elif (self._is_read_rgb) and (self._is_read_depth):  # rgb and depth.
            # If both `rgb` and `depth`,
            # then their filenames are supposed to be the same.
            rgb_base_names = get_filenames(rgb_folder, is_base_name=True)
            rgb_names = [rgb_folder + s for s in rgb_base_names]
            depth_names = [depth_folder + s for s in rgb_base_names]
        self._rgb_names = rgb_names
        self._depth_names = depth_names

        # -- Read camera info.
        if self._is_read_camera_info:
            camera_info = MyCameraInfo(camera_info_file_path)
        else:
            camera_info = None
        self._camera_info = camera_info

        # -- Variables
        self._cnt_images = 0

    def is_pub_rgb(self):
        return self._is_pub_rgb

    def is_pub_depth(self):
        return self._is_pub_depth

    def is_pub_camera_info(self):
        return self._is_pub_camera_info

    def is_pub_cloud(self):
        return self._is_pub_cloud


def main(config):

    # -- Set data loader
    rgbd_data = RgbdDataLoader(config)

    # -- Set publisher
    ns = config["ros_topic_namespace"] + "/"
    frame_id = config["frame_id"]

    topic_name = ns + config["rgb"]["ros_topic"]
    pub_rgb = ColorImagePublisher(topic_name)

    topic_name = ns + config["depth"]["ros_topic"]
    pub_depth = DepthImagePublisher(topic_name)

    topic_name = ns + config["camera_info"]["ros_topic"]
    pub_camera_info = CameraInfoPublisher(topic_name)

    pub_cloud = None  # TODO

    # -- Loop and publish
    is_loop_forever = config["is_loop_forever"]
    loop_rate = rospy.Rate(config["pub_frequency"])

    while not rospy.is_shutdown():

        # -- Read next data.
        if not rgbd_data.has_data():
            if is_loop_forever:  # Reset data loader.
                rgbd_data.reset()
            else:  # Stop program.
                break
        rgb, depth, camera_info, cloud = rgbd_data.read_next_data()

        # -- Publish data.
        rospy.loginfo("=================================================")

        # rgb.
        if rgbd_data.is_pub_rgb():
            pub_rgb.publish(rgb, frame_id)
            rospy.loginfo("Publish rgb image, "
                          "shape = {}.".format(rgb.shape))

        # depth.
        if rgbd_data.is_pub_depth():
            pub_depth.publish(depth, frame_id)
            rospy.loginfo("Publish depth image, "
                          "shape = {}.".format(depth.shape))

        # camera_info.
        if rgbd_data.is_pub_camera_info():
            pub_camera_info.publish(
                camera_info.width(),
                camera_info.height(),
                camera_info.intrinsic_matrix(),
                frame_id)
            rospy.loginfo("Publish camera info.")

        # point_cloud.
        if rgbd_data.is_pub_cloud():
            pub_cloud.publish(cloud)
            rospy.loginfo("Publish point cloud, "
                          "{} points.".format(3))

        # -- Wait for publish.
        loop_rate.sleep()


if __name__ == '__main__':
    node_name = "pub_rgbd_and_cloud"
    rospy.init_node(node_name)
    args = parse_command_line_argumetns()
    config = read_config_file(args.config_file)
    main(config)
    rospy.logwarn("Node `{}` stops.".format(node_name))
