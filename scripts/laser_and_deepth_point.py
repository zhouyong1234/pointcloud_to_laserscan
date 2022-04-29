#!/usr/bin/env python
# -*-coding:utf-8-*-

import rospy
import thread, threading
import time
import message_filters
import numpy as np
from sensor_msgs.msg import Joy, LaserScan, PointCloud2
from base_msgs.msg import DockInfraRed
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg

		
class laserTracker:
    def __init__(self):

        self.scan_pub = rospy.Publisher("/combined_scan", LaserScan, queue_size=10)
        self.scan_sub = message_filters.Subscriber("/scan", LaserScan, queue_size=1)
        self.camera_01_sub = message_filters.Subscriber("/camera_01/scan", LaserScan, queue_size=1)
        self.camera_02_sub = message_filters.Subscriber("/camera_02/scan", LaserScan, queue_size=1)

        self.sync = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.camera_01_sub, self.camera_02_sub], 1, 5, allow_headerless=False)

        self.sync.registerCallback(self.callback)
        
    def callback(self, scan_data, camera_01_data, camera_02_data):

        # rospy.loginfo("Begin to combine scan data and depth data......")

        # 先得到激光数据和深度摄像头数据
        scan_data_list = list(scan_data.ranges)
        camera_01_data_list = list(camera_01_data.ranges)
        camera_02_data_list = list(camera_02_data.ranges)


        # 默认为激光数据
        scan_data_combined = LaserScan()
        scan_data_combined.header.frame_id = "laser"
        scan_data_combined.header.stamp = rospy.Time.now()
        scan_data_combined.angle_min = 0.0174534954131
        scan_data_combined.angle_max = 6.28318548203
        scan_data_combined.angle_increment = 0.00435422640294
        scan_data_combined.time_increment = 5e-05
        scan_data_combined.scan_time = 0.0645535737276
        scan_data_combined.range_min = 0.15
        scan_data_combined.range_max = 25.0
        scan_data_combined.ranges = list(scan_data.ranges)
        scan_data_combined.intensities = []

        # 将激光数据25度-90度和270度-335度的数据和深度摄像头数据对比,25-90对应camera_02, 270-335对应camera_01
        for i, j, k in zip(range(100, 372), range(1068, 1340), range(0, 272)):
            if list(scan_data.ranges)[i] > list(camera_02_data.ranges)[k]:
                scan_data_combined.ranges[i] = camera_02_data.ranges[k]
            if list(scan_data.ranges)[j] > list(camera_01_data.ranges)[k]:
                scan_data_combined.ranges[j] = camera_01_data.ranges[k]


        for i, j in zip(range(100,372), range(1068, 1340)):
            print(scan_data_combined.ranges[i])
            print(scan_data_combined.ranges[j])

        self.scan_pub.publish(scan_data_combined)


if __name__ == '__main__':
    rospy.init_node('laser_tracker')
    tracker = laserTracker()
    rospy.spin()


