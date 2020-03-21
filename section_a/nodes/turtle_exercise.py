#!/usr/bin/env python

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

kw = 2.0
kv = 2.0
rospy.init_node('turtle_exercise', anonymous=True)

abhi = Pose()
abhi.x = 10.0
abhi.y = 10.0
turt = Pose()
turt.x = 5.4
turt.y = 5.4
vec = [abhi.x-turt.x, abhi.y-turt.y, 0]
vec[2] = math.atan2(vec[1], vec[0])

def abhi_cb(data):
    global abhi
    global vec
    abhi = data
    vec = [abhi.x-turt.x, abhi.y-turt.y, vec[2]]
    vec[2] = math.atan2(vec[1], vec[0])

def turt_cb(data):
    global turt
    global vec
    turt = data
    vec = [abhi.x-turt.x, abhi.y-turt.y, vec[2]]
    vec[2] = math.atan2(vec[1], vec[0])

def euclid(a,b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5

def vel_pub():
    msg = Twist()
    global pub
    if  abs(vec[2] - turt.theta) > 0.01:
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
    
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = kw * (vec[2]-turt.theta)
    
    else:
        msg.linear.x = kv * (euclid(turt, abhi) - 2)
        msg.linear.y = kv * (euclid(turt, abhi) - 2)
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
    
    pub.publish(msg)

rospy.Subscriber("/turtle1/pose", Pose, turt_cb)
rospy.Subscriber("/abhiyaan/pose", Pose, abhi_cb)	
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

while (vec[0]**2 + vec[1]**2)**0.5 > 2.05:
    try:
        vel_pub()
    except KeyboardInterrupt:
        break
#rospy.spin()
