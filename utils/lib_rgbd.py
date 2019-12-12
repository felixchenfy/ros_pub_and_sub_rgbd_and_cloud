''' 
Computer vision related functions and classes:
    * class MyCameraInfo
    * def create_open3d_point_cloud_from_rgbd
'''

import cv2
import simplejson
import open3d
import numpy as np


class MyCameraInfo():

    def __init__(self,
                 camera_info_file_path=None,
                 ros_camera_info=None):
        if camera_info_file_path is not None:
            data = read_json_file(camera_info_file_path)
        elif ros_camera_info is not None:
            data = self._from_ros_camera_info(ros_camera_info)
        else:
            raise RuntimeError("Invalid input for MyCameraInfo().")
        self._width = int(data["width"])  # int.
        self._height = int(data["height"])  # int.
        self._intrinsic_matrix = data["intrinsic_matrix"]  # list of float.
        # The list extracted from the matrix **column by column** !!!.
        # If the intrinsic matrix is:
        # [fx,  0, cx],
        # [ 0, fy, cy],
        # [ 0,  0,  1],
        # Then, self._intrinsic_matrix = [fx, 0, 0, 0, fy, 0, cx, cy, 1]

    def resize(self, ratio):
        r0, c0 = self._height, self._width
        if not (is_int(r0*ratio) and is_int(c0*ratio)):
            raise RuntimeError(
                "Only support resizing image to an interger size.")
        self._width = int(ratio * self._width)
        self._height = int(ratio * self._height)
        self._intrinsic_matrix[:-1] = [x*ratio
                                       for x in self._intrinsic_matrix[:-1]]

    def width(self):
        return self._width

    def height(self):
        return self._height

    def intrinsic_matrix(self):
        return self._intrinsic_matrix

    def get_cam_params(self):
        ''' Get all camera parameters. 
        Notes: intrinsic_matrix:
            [0]: fx, [3]   0, [6]:  cx
            [1]:  0, [4]: fy, [7]:  cy
            [2]:  0, [5]   0, [8]:   1
        '''
        im = self._intrinsic_matrix
        row, col = self._height, self._width
        fx, fy, cx, cy = im[0], im[4], im[6], im[7]
        return row, col, fx, fy, cx, cy

    def to_open3d_format(self):
        ''' Convert camera info to open3d format of `class open3d.camera.PinholeCameraIntrinsic`.
        Reference: http://www.open3d.org/docs/release/python_api/open3d.camera.PinholeCameraIntrinsic.html
        '''
        row, col, fx, fy, cx, cy = self.get_cam_params()
        open3d_camera_info = open3d.camera.PinholeCameraIntrinsic(
            col, row, fx, fy, cx, cy)
        return open3d_camera_info

    def _from_ros_camera_info(self, ros_camera_info):
        K = ros_camera_info.K  # ROS is row majored.
        # However, here I'm using column majored.
        data = {
            "width": ros_camera_info.width,
            "height": ros_camera_info.height,
            "intrinsic_matrix": [
                K[0], K[3], K[6],
                K[1], K[4], K[7],
                K[2], K[5], K[8]]
        }
        return data


def create_open3d_point_cloud_from_rgbd(
        color_img, depth_img,
        cam_info,
        depth_unit=0.001,
        depth_trunc=3.0):
    ''' Create pointcreate_open3dpoint_cloud_from_rgbd cloud of open3d format, given opencv rgbd images and camera info.
    Arguments:
        color_img {np.ndarry, np.uint8}:
            3 channels of BGR. Undistorted.
        depth_img {np.ndarry, np.uint16}:
            Undistorted depth image that matches color_img.
        cam_info {CameraInfo}
        depth_unit {float}:
            if depth_img[i, j] is x, then the real depth is x*depth_unit meters.
        depth_trunc {float}:
            Depth value larger than ${depth_trunc} meters
            gets truncated to 0.
    Output:
        open3d_point_cloud {open3d.geometry.PointCloud}
            See: http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html
    Reference:
    '''

    # Create `open3d.geometry.RGBDImage` from color_img and depth_img.
    rgbd_image = open3d.geometry.RGBDImage.create_from_color_and_depth(
        color=open3d.geometry.Image(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)),
        depth=open3d.geometry.Image(depth_img),
        depth_scale=1.0/depth_unit,
        convert_rgb_to_intensity=False)

    # Convert camera info to `class open3d.camera.PinholeCameraIntrinsic`.
    pinhole_camera_intrinsic = cam_info.to_open3d_format()

    # Project image pixels into 3D world points.
    # Output type: `class open3d.geometry.PointCloud`.
    open3d_point_cloud = open3d.geometry.PointCloud.create_from_rgbd_image(
        image=rgbd_image,
        intrinsic=pinhole_camera_intrinsic)

    return open3d_point_cloud


def read_json_file(file_path):
    with open(file_path, 'r') as f:
        data = simplejson.load(f)
    return data


def is_int(num):
    ''' Is floating number very close to a int. '''
    # print(is_int(0.0000001)) # False
    # print(is_int(0.00000001)) # True
    return np.isclose(np.round(num), num)
