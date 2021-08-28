#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#Lidar Preprocess variables
past_ranges_t1 = past_ranges_t2 = past_ranges_t3 = past_ranges_t4 = current_ranges_t5 = np.zeros(1080)

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # converting list to array
        #arr = numpy.array(lst)
        global past_ranges_t1, past_ranges_t2, past_ranges_t3, past_ranges_t4, current_ranges_t5
        past_ranges_t1 = past_ranges_t2
        past_ranges_t2 = past_ranges_t3
        past_ranges_t3 = past_ranges_t4
        past_ranges_t4 = current_ranges_t5
        current_ranges_t5 = ranges
        proc_ranges = 0.5 * current_ranges_t5 + 0.25 * past_ranges_t4 + 0.15 * past_ranges_t3 + 0.07 * past_ranges_t2 + 0.03 * past_ranges_t1
        proc_ranges[proc_ranges > 4.0] = 4.0
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        return None

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
