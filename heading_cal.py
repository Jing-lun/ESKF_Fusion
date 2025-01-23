#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import NavSatFix

def callback1(data):
    global lat1, lon1
    lat1 = data.latitude
    lon1 = data.longitude

def callback2(data):
    global lat2, lon2
    lat2 = data.latitude
    lon2 = data.longitude
    calculate_heading()

def calculate_heading():
    # Constants for conversion
    R = 6371000  # Earth radius in meters
    x = (lon2 - lon1) * math.cos(0.5 * (lat2 + lat1) * math.pi/180) * R * math.pi/180
    y = (lat2 - lat1) * R * math.pi/180
    
    # Heading calculation
    heading = math.atan2(x, y) * 180/math.pi
    rospy.loginfo("Heading: %f degrees", heading)

def listener():
    rospy.init_node('gnss_heading_calculator', anonymous=True)
    rospy.Subscriber("device/gnss_base/fix", NavSatFix, callback1)
    rospy.Subscriber("device/gnss_rover/fix", NavSatFix, callback2)
    print("I can see the topics")
    rospy.spin()

if __name__ == '__main__':
    lat1 = lon1 = lat2 = lon2 = 0
    print("I can see the topics 1")
    listener()

