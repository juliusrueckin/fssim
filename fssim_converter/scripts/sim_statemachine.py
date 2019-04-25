#!/usr/bin/python 
import sys 
import numpy as np 
import math 
import time 
import rospy 
from std_msgs.msg import Float64 
from std_msgs.msg import Int32 
from std_msgs.msg import Bool 
from std_msgs.msg import Float64MultiArray 
from nav_msgs.msg import Odometry 
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from geometry_msgs.msg import PoseStamped 
from tf.msg import tfMessage 
from tf.transformations import euler_from_quaternion 
from tf.transformations import quaternion_from_euler 
from custom_msgs.msg import Mission

rospy.init_node('sim_startup_car', anonymous=True) 


pub_state = rospy.Publisher("/statemachine/state", Int32, queue_size=1) 
pub_mission = rospy.Publisher("/statemachine/mission", Mission, queue_size=1)
statemachine_enum = rospy.get_param("/statemachine_enum") 
mission_enum = rospy.get_param("/mission_enum") 


time.sleep(1) 
pub_state.publish(Int32(statemachine_enum["asoff"])) 
time.sleep(1) 
pub_mission.publish(Mission(None, Mission.GATE))
time.sleep(1) 
pub_state.publish(Int32(statemachine_enum["ready"])) 
time.sleep(3) 
pub_state.publish(Int32(statemachine_enum["driving"])) 
time.sleep(120)
pub_state.publish(Int32(statemachine_enum["ebs_triggered"]))

