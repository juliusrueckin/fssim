#!/usr/bin/python2
import rospy
import ros_numpy
import numpy as np
import math
from custom_msgs.msg import ConeArray, Cone, OdometryState
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from fssim_common.msg import State

from geometry_msgs.msg import Pose2D
from tf.transformations import quaternion_from_euler


pub_cones = rospy.Publisher('/perception/cones/planning', ConeArray, queue_size=1)
pub_state = rospy.Publisher('/stateestimation/odometry', Odometry, queue_size=1)
pub_state_v = rospy.Publisher('/stateestimation/odometry_v', OdometryState, queue_size=1)


state = Pose2D()


def callback_cones(data):
    global state

    lidar_data = ros_numpy.numpify(data)
    # lidar_data_intensities = np.array([x[3] for x in lidar_data])
    # cov = np.array([np.array([x[4], x[5], x[6], x[7]]) for x in lidar_data])
    # col = np.array([np.array([x[8], x[9], x[10], x[11]]) for x in lidar_data])
    lidar_data = np.array([np.array([x[0], x[1], x[2], x[8], x[9], x[10], x[11]]) for x in lidar_data])

    # print("points")
    # print(lidar_data)
    # print("intesities")
    # print(lidar_data_intensities)
    # print("cov")
    # print(cov)
    # print("col")
    # print(col)

    msg = ConeArray()
    msg.header = data.header
    msg.header.frame_id = 'map'

    for row in lidar_data:
        c = Cone()
        c.position.x = math.cos(state.theta) * row[0] - math.sin(state.theta) * row[1] + state.x
        c.position.y = math.sin(state.theta) * row[0] + math.cos(state.theta) * row[1] + state.y
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
    global state

    state = Pose2D()
    state.x = data.x
    state.y = data.y
    state.theta = data.yaw

    msg_v = OdometryState()
    msg_ros = Odometry()

    msg_v.header = data.header
    msg_ros.header = data.header

    [x, y, z, w] = quaternion_from_euler(0, 0, state.theta)

    msg_v.state.pose.pose.position.x = data.x
    msg_v.state.pose.pose.position.y = data.y
    msg_v.state.pose.pose.orientation.x = x
    msg_v.state.pose.pose.orientation.y = y
    msg_v.state.pose.pose.orientation.z = z
    msg_v.state.pose.pose.orientation.w = w

    msg_ros.pose.pose.position.x = data.x
    msg_ros.pose.pose.position.y = data.y
    msg_ros.pose.pose.orientation.x = x
    msg_ros.pose.pose.orientation.y = y
    msg_ros.pose.pose.orientation.z = z
    msg_ros.pose.pose.orientation.w = w

    v = np.array([data.vx, data.vy])
    msg_v.velocity = np.sqrt(np.sum(np.square(v)))

    pub_state.publish(msg_ros)
    pub_state_v.publish(msg_v)


if __name__ == '__main__':

    rospy.loginfo("Launched planner converter node")
    rospy.init_node('controller_converter', anonymous=False)
    rospy.Subscriber('/lidar/cones', PointCloud2, callback_cones)
    rospy.Subscriber('/fssim/base_pose_ground_truth', State, callback_state)

    rospy.spin()
