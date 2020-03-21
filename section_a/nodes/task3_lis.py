#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(data.data)
    
def listener():

    rospy.init_node('task3_listener', anonymous=True)

    rospy.Subscriber("/welcome_message", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
