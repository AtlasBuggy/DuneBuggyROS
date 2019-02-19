#!/usr/bin/python

import rospy
import pandas as pd
import string
from std_msgs.msg import String

def callback(data):
    imuData = pd.DataFrame()
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #AX = (float(data.data[data.data.index("A")+1:data.data.index("B")]))
    #AY = (float(data.data[data.data.index("B")+1:data.data.index("C")]))
    #AZ = (float(data.data[data.data.index("C")+1:data.data.index("D")])) 
    #GX = (float(data.data[data.data.index("D")+1:data.data.index("E")]))
    #GY = (float(data.data[data.data.index("E")+1:data.data.index("F")]))
    #GZ = (float(data.data[data.data.index("F")+1:data.data.index("G")]))
    dataLines = data.data.split()
    line = []
    for i in range(len(dataLines)):
	if ":" in dataLines[i]: dataLines[i] = dataLines[i][0]
    #for dataLine in dataLines:
    #    line.append({dataLine[0][0]:dataLine[2:]})
    #imuData.append(line) 
    #if imuData.count() > 1000: 
    #	imuData = imuData.to_csv(r"/opt/ros/kinetic/share/imu/imuData.csv",index=False,header=True)
    #	return
    print(dataLines)
def listener(): # nodes are unique in ROS
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("imu", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener() 





