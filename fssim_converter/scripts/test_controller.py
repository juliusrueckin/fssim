#!/usr/bin/python2
import rospy
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from fssim_common.msg import Cmd

from tf.transformations import quaternion_from_euler


pub_control = rospy.Publisher('/fssim/cmd', Cmd, queue_size=1)

speed = 0.05
steer = 0.0


def callback_steer(data):
    global steer
    global speed

    steer = data.data

    msg = Cmd()
    msg.dc = speed
    msg.delta = steer
    pub_control.publish(msg)


def callback_speed(data):
    global steer
    global speed

    speed = data.data

    msg = Cmd()
    msg.dc = speed
    msg.delta = steer
    pub_control.publish(msg)


# 1) Add callback that listens to our control commands and publishes either on 
# /fssim/cmd (fssim_common/Cmd) or 
# /control/pure_pursuit/control_command (fsd_common_msgs/ControlCommand)
# they seem to do the same thing. Note that the later one still works when
# the control_meta node was never launched

# 2) somehow convince our controller to send commands (because of the states/ebs/go_signal stuff)
# should probably change the controller.py script such that given some param it starts right away


if __name__ == '__main__':

    rospy.loginfo("Launched controller converter node")
    rospy.init_node('controller_converter', anonymous=False)

    rospy.Subscriber('/control/steer', Float64, callback_steer)
    rospy.Subscriber('/control/speed', Float64, callback_speed)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        continue
