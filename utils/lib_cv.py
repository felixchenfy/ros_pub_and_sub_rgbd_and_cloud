''' 
Computer vision related functions and classes:
    * class MyCameraInfo
    * def create_open3d_point_cloud_from_rgbd
'''

import cv2
import simplejson
import open3d


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

    def to_open3d_format(self):
        ''' Convert camera info to open3d format.
        See `class open3d.camera.PinholeCameraIntrinsic`
        at: http://www.open3d.org/docs/release/python_api/open3d.camera.PinholeCameraIntrinsic.html#open3d.camera.PinholeCameraIntrinsic

        Notes: intrinsic_matrix:
            0: fx, 1   0, 2:  cx
            3:  0, 4: fy, 5:  cy
            6:  0, 7   0, 8:   1
        '''
        im = self._intrinsic_matrix
        open3d_camera_info = open3d.camera.PinholeCameraIntrinsic(
            width=self._width,
            height=self._height,
            fx=im[0], fy=im[4],
            cx=im[2], cy=im[5])
        return open3d_camera_info


def create_open3d_point_cloud_from_rgbd(
        color_img, depth_img,
        camera_info,
        depth_unit=0.001,
        depth_trunc=3.0):
    ''' Create pointcreate_open3dpoint_cloud_from_rgbd cloud of open3d format, given opencv rgbd images and camera info.
    Arguments:
        color_img {np.ndarry, np.uint8}: 
            3 channels of BGR. Undistorted.
        depth_img {np.ndarry, np.uint16}: 
            Undistorted depth image that matches color_img.
        camera_info {MyCameraInfo}
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
    # http://www.open3d.org/docs/0.7.0/python_api/open3d.geometry.create_rgbd_image_from_color_and_depth.html#open3d.geometry.create_rgbd_image_from_color_and_depth
    rgbd_image = open3d.create_rgbd_image_from_color_and_depth(
        color=open3d.Image(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)),
        depth=open3d.Image(depth_img),
        depth_scale=1.0/depth_unit,
        convert_rgb_to_intensity=False)

    # Convert camera info to `class open3d.camera.PinholeCameraIntrinsic`.
    # http://www.open3d.org/docs/release/python_api/open3d.camera.PinholeCameraIntrinsic.html
    pinhole_camera_intrinsic = camera_info.to_open3d_format()

    # Project image pixels into 3D world points.
    # Output type: `class open3d.geometry.PointCloud`.
    # http://www.open3d.org/docs/0.6.0/python_api/open3d.geometry.create_point_cloud_from_rgbd_image.html#open3d.geometry.create_point_cloud_from_rgbd_image
    open3d_point_cloud = open3d.create_point_cloud_from_rgbd_image(
        image=rgbd_image,
        intrinsic=pinhole_camera_intrinsic)

    return open3d_point_cloud


def read_json_file(file_path):
    with open(file_path, 'r') as f:
        data = simplejson.load(f)
    return data
