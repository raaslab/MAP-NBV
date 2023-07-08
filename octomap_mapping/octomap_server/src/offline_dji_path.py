#!/usr/bin/env python
import rospy
import tf
import csv

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import time

def talker():
    rospy.init_node('offline_dji_path_node')

    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "world"

    path = Path()

    path.header = h
    pose = PoseStamped()
    #pose.header = data.header
    #pose.pose = data.pose.pose
    #path.poses.append(pose)
    #path_pub.publish(path)

    reader = csv.reader(open('/home/user/dji_path.csv', 'rb'), delimiter=",")
    count = 0
    for row in reader:
        if count == 0:
            count = 1
            continue
        pose = PoseStamped()
        pose.header = h
        pose.pose.position.x = float(row[0])
        pose.pose.position.y = float(row[1])
        pose.pose.position.z = float(row[2]) - 0.5
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        path.poses.append(pose)
        print(row)


    path_pub = rospy.Publisher('/offline_dji_path_node', Path, queue_size=10)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        path_pub.publish(path)
        #rate.sleep()
        time.sleep(0.5)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass