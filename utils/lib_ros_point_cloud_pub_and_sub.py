'''
Publisher/Subscriber for point cloud.
'''

from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d

from sensor_msgs.msg import PointCloud2

import rospy
import queue


class PointCloudPublisher(object):

    def __init__(self, topic_name):
        self._pub = rospy.Publisher(topic_name, PointCloud2, queue_size=5)

    def publish(self, cloud, cloud_format="open3d", frame_id="head_camera"):
        if cloud_format == "open3d":
            cloud = convertCloudFromOpen3dToRos(cloud, frame_id)
        else:  # ROS cloud: Do nothing.
            pass
        self._pub.publish(cloud)


class PointCloudSubscriber(object):

    def __init__(self, topic_name, queue_size=2):
        self._sub = rospy.Subscriber(
            topic_name, PointCloud2, self._callback_of_pcd_subscriber)
        self._clouds_queue = queue.Queue(maxsize=queue_size)

    def get_cloud(self):
        ''' Get the next cloud subscribed from ROS topic. 
            Convert it to open3d format and then return. '''
        if not self.has_cloud():
            return None
        ros_cloud = self._clouds_queue.get(timeout=0.05)
        open3d_cloud = convertCloudFromRosToOpen3d(ros_cloud)
        return open3d_cloud

    def has_cloud(self):
        return self._clouds_queue.qsize() > 0

    def _callback_of_pcd_subscriber(self, ros_cloud):
        ''' Save the received point cloud into queue.
        '''
        if self._clouds_queue.full():  # If queue is full, pop one.
            cloud_to_discard = self._clouds_queue.get(timeout=0.001)
        self._clouds_queue.put(ros_cloud, timeout=0.001)  # Push cloud to queue
