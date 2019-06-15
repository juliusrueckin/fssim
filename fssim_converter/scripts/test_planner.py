#!/usr/bin/python2
import rospy
import ros_numpy
import numpy as np
import math
from custom_msgs.msg import ConeArray, Cone
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from fssim_common.msg import State

from geometry_msgs.msg import Pose2D
from tf.transformations import quaternion_from_euler


pub_cones = rospy.Publisher('/perception/lidar/cones', ConeArray, queue_size=1)
pub_state = rospy.Publisher('/stateestimation/odometry', Odometry, queue_size=1)



def callback_cones(data):

    lidar_data = ros_numpy.numpify(data)
    lidar_data = np.array([np.array([x[0], x[1], x[2], x[8], x[9], x[10], x[11]]) for x in lidar_data])

    msg = ConeArray()
    msg.header = data.header
    msg.header.frame_id = 'pandar'

    for row in lidar_data:
        c = Cone()
        c.position.x = row[0]
        c.position.y = row[1] 
        i = np.argmax(np.array(row[3:7]))
        if i == 0:
            c.type = Cone.LEFT
        elif i == 1:
            c.type = Cone.RIGHT
        else:
            c.type = Cone.OTHER
        msg.cones.append(c)

    pub_cones.publish(msg)


def callback_state(data):

    msg_ros = Odometry()

    msg_ros.header = data.header

    [x, y, z, w] = quaternion_from_euler(0, 0, data.yaw)

    msg_ros.pose.pose.position.x = data.x
    msg_ros.pose.pose.position.y = data.y
    msg_ros.pose.pose.orientation.x = x
    msg_ros.pose.pose.orientation.y = y
    msg_ros.pose.pose.orientation.z = z
    msg_ros.pose.pose.orientation.w = w

    v = np.array([data.vx, data.vy])
    msg_ros.twist.twist.linear.x = np.sqrt(np.sum(np.square(v)))

    pub_state.publish(msg_ros)


if __name__ == '__main__':

    rospy.loginfo("Launched planner converter node")
    rospy.init_node('controller_converter', anonymous=False)
    rospy.Subscriber('/lidar/cones', PointCloud2, callback_cones)
    rospy.Subscriber('/fssim/base_pose_ground_truth', State, callback_state)

    rospy.spin()
