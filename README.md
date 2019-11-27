# ROS rgbd and cloud publisher/subscriber 

This package has two nodes:
1. `pub_rgbd_and_cloud.py`
2. `sub_rgbd_and_cloud.py`

What data you can publish:
1. rgb image (read from a folder).
2. depth image (read from a folder).
3. camera_info (read from a file, without distortion).
4. point cloud (generated from rgb, depth, and camera_info)

You can publish any combination of the above data (e.g., rgb only, or rgbd + depth + point cloud) as long as the data paths are valid, and the following requirements are satisfied:
  * If publishing both `rgb` and `depth`, their filename should be the same in their respective folder.
  * If publishing `point cloud`, the data paths of all `rgb` and `depth` and `camera_info` needs to be valid, because the point cloud is generated from them.

# Example of usage

* **Publish tf (Not necessary):**
  For better visualization in rviz, you may set camera pose at origin by:
  ``` bash
  $ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map head_camera 10
  ```
  The camera `frame_id` is set as `head_camera` in [config/publisher_settings.yaml](config/publisher_settings.yaml).


* **Publish data:**
  The test data are already included in `data/` folder, you can directly run:
  ``` bash
  $ rosrun ros_pub_and_sub_rgbd_and_cloud pub_rgbd_and_cloud.py \
    --config_file config/publisher_settings.yaml
  ```

  The published ROS topics are:
  ``` bash
  TODO
  /my_test/camera_info
  /my_test/depth
  /my_test/rgb
  ```

* **View result in rviz:**
  ``` bash
  $ roslaunch ros_pub_and_sub_rgbd_and_cloud run_rviz.launch 
  ```

* **Subscribe data:**
  ``` bash
  $ rosrun ros_pub_and_sub_rgbd_and_cloud sub_rgbd_and_cloud.py \
    --config_file config/publisher_settings.yaml
  ```


# Configuration

See [config/publisher_settings.yaml](config/publisher_settings.yaml)
``` yaml
# Configuration for `pub_rgbd_and_cloud.py`.

ros_topic_namespace: "my_test"
frequency: 0.5 # How many data to publish per second.
enable_loop: True # After reading all images from the folder, loop back to the 1st one.

rgb:
  is_publish: True
  folder: "/home/feiyu/catkin_ws/src/ros_pub_and_sub_rgbd_and_clouddata/color/"
  ros_topic: "rgb"

depth:
  is_publish: True
  folder: "/home/feiyu/catkin_ws/src/ros_pub_and_sub_rgbd_and_clouddata/depth/"
  ros_topic: "depth"

camera_info:
  is_publish: True
  file: "/home/feiyu/catkin_ws/src/ros_pub_and_sub_rgbd_and_cloud/config/cam_params_realsense.json"
  ros_topic: "camera_info"

point_cloud:
  is_publish: True
  ros_topic: "point_cloud"
```

# Dependencides
The package is written under: Ubuntu 18.04, ROS melodic, Python 2.7.

The main dependencies are:
* cv2
* pcl (for ROS point cloud message type.)
* open3d (for creating point cloud from rgbd images.)

Installation commands:
* Install pcl (Might be installed by default when installing ROS melodic):
    http://wiki.ros.org/pcl_ros  
    ``` bash
    sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl  
    sudo apt-get update  
    sudo apt-get install libpcl-all  
    sudo apt-get install ros-melodic-pcl-ros  
    sudo apt-get install ros-melodic-pcl-conversions    
    ```

* Open3D  
    See two official installation tutorials: [this](http://www.open3d.org/docs/getting_started.html) and [this](http://www.open3d.org/docs/compilation.html).
    ``` bash
    pip install --user open3d
    ```
