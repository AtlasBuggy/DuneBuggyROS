#! usr/bin/env python3
import rospy
from std_msgs.msg import Imu, NavSatFix, uint8, Quaternion
import serial

"""
Receives a message from the arduino through serial and parses the sensor
data from that message

message type:   <latitude:  double>\t<longitude: double>\t<altitude: double>\t
                <update_flag: int>\t
                <ch1: int>\t<ch2: int>\t<ch3: int>\t
                <quatx: double> \t<quaty: double>\t<quatz: double>\t
                <quatw: double>\n

1. Read line from serial
2. Parse string into:
    IMU comp
    GPS comp
    RF comp
3. Publish ROS messages:
    /imu
    /gps
    /ch1
    ...
    /ch6
"""

"""
Datatypes:

IMU:
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance

GPS:
    uint8 COVARIANCE_TYPE_UNKNOWN=0
    uint8 COVARIANCE_TYPE_APPROXIMATED=1
    uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
    uint8 COVARIANCE_TYPE_KNOWN=3
    std_msgs/Header header
    sensor_msgs/NavSatStatus status
    float64 latitude
    float64 longitude
    float64 altitude
    float64[9] position_covariance
    uint8 position_covariance_type

UInt8:
    uint8 data
"""
def parse_GPS(data_string):
    # parses GPS part of string in the form:
    # <latitude: double>\t<longitude: double>\t<altitude: double>\t
    # <update_flag: int>\t
    GPS_end = 4
    data = line.split("\t")[:GPS_end]
    GPS_msg = NavSatFix()
    GPS_msg.latitude = data[0]
    GPS_msg.longitude = data[1]
    GPS_msg.altitude = data[2]
    GPS_msg.status.status = data[3] # update flag
    return GPS_msg


def parse_channel(data_string, channel_number):
    # parses channel part of the string of the form:
    # <ch1: int>\t<ch2: int>\t<ch3: int>\t
    channels_start = 4
    channels_end = 8
    data = line.split("\t")[channels_start:channels_end]
    channel_msg = UInt8()
    channel_msg.data = data[channel_number]
    return channel_msg


def parse_IMU(data_string):
    # parses IMU part of the string of the form
    # <quatx: double>\t<quaty: double>\t<quatz: double>\t<quatw: double>\n
    IMU_start = 8
    data = line.split("\t")[IMU_start:]
    IMU_msg.orientation.x = data[0]
    IMU_msg.orientation.y = data[1]
    IMU_msg.orientation.z = data[2]
    IMU_msg.orientation.w = data[3]
    return IMU_msg


def sensor_publisher(ser):
    channel_number = 4

    # define the publishers(IMU, GPS, ch1-ch6)
    imu_pub = rospy.Publisher('IMU', Imu, queue_size=10)
    gps_pub = rospy.Publisher('GPS', NavSatFix, queue_size=10)
    channel_publishers = []
    for i in range(1, channel_number):
        channel_publishers.append(rospy.Publisher('ch' + str(i), UInt8, queue_size=10))

    rospy.init_node('sensor_info', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # read the line from the arduino, parse the data for each message
        # TODO: haven't tested readline so I'm not sure this works
        line = ser.readline()
        imu_msg = parse_IMU(line)
        gps_msg = parse_GPS(line)
        channel_msgs = []
        for i in range(1, channel_number):
            channel_msgs.append(parse_channel(line, i))

        # publish all the data
        rospy.loginfo(imu_msg, gps_msg)
        imu_pub.publish(imu_msg)
        gps_pub.publish(gps_msg)
        for i in range(1, channel_number):
            pub = channel_publishers[i]
            msg = channel_msgs[i]
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    # initialize the serial connection to the arduino
    # TODO make this serial stuff work
    ser = serial.Serial()
    ser.baudrate = 115200
    ser.port = "some port name"
    ser.timeout = 1
    ser.open()
    print(ser.name)

    try:
        sensor_publisher(ser)
    except rospy.ROSInterruptException:
        ser.close()
        pass

