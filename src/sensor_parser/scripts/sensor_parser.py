#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt32
import serial

# Receives a message from the arduino through serial and parses the sensor
# data from that message

# Message Format:
#   <quatx: double> \t<quaty: double>\t<quatz: double>\t<quatw: double>
#   <latitude:  double>\t<longitude: double>\t<altitude: double>\t
#   <ch1: int>\t<ch2: int>\t<ch3: int>\t<ch4: int>\t<ch5: int>\t<ch6: int>\n

def parse_GPS(data):
    # parses GPS part of string in the form:
    GPS_msg = NavSatFix()
    GPS_msg.latitude = float(data[0])
    GPS_msg.longitude = float(data[1])
    GPS_msg.altitude = float(data[2])
    # GPS_msg.status.status = int(data[3]) # update flag
    return GPS_msg


def parse_channel(data, channel_number):
    # parses channel part of the string of the form:
    channel_msg = UInt32()
    channel_msg.data = int(data[channel_number])
    return channel_msg


def parse_IMU(data):
    # parses IMU part of the string of the form
    IMU_msg = Imu()
    IMU_msg.orientation.x = float(data[0])
    IMU_msg.orientation.y = float(data[1])
    IMU_msg.orientation.z = float(data[2])
    IMU_msg.orientation.w = float(data[3])
    return IMU_msg


def parse_sensors(ser):
    number_channels = 6
    number_params = 14
    GPS_start = 4
    IMU_start = 0
    channel_start = 7
    GPS_length = 3
    IMU_length = 4
    channel_length = 6

    # define the publishers(IMU, GPS, ch1-ch6)
    imu_pub = rospy.Publisher('IMU', Imu, queue_size=10)
    gps_pub = rospy.Publisher('GPS', NavSatFix, queue_size=10)
    channel_publishers = []
    for i in range(number_channels):
        channel_publishers.append(rospy.Publisher('ch' + str(i+1), UInt32,
                                  queue_size=10))

    rospy.init_node('sensor_info', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # read the line from the arduino, parse the data for each message
        line = ser.readline()

        data = line.split('\t')

        # if the full data didn't come though, ignore
        if len(data) != number_params:
            continue

        # parse the strings into ros messages
        imu_msg = parse_IMU(data[IMU_start : IMU_start + IMU_length])
        gps_msg = parse_GPS(data[GPS_start : GPS_start + GPS_length])
        channel_msgs = []
        for i in range(number_channels):
            channel_slice = data[channel_start : channel_start + channel_length]
            channel_msgs.append(parse_channel(channel_slice, i))

        # publish on all the topics
        imu_pub.publish(imu_msg)
        gps_pub.publish(gps_msg)
        for i in range(number_channels):
            pub = channel_publishers[i]
            msg = channel_msgs[i]
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    # initialize the serial connection to the arduino
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55834323633351F030F1-if00"
    ser.timeout = 1
    ser.open()

    try:
        parse_sensors(ser)
    except rospy.ROSInterruptException:
        ser.close()

