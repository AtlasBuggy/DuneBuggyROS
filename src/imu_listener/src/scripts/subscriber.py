#!/usr/bin/python

import rospy
import pandas as pd
import string
from std_msgs.msg import String

def callback(data):
    imuData = pd.DataFrame()
    dataLines = data.data.split()
    line = []
    for i in range(len(dataLines)):
	if ":" in dataLines[i]: dataLines[i] = dataLines[i][0]
    print(dataLines)

def listener(): # nodes are unique in ROS
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("imu", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener() 





