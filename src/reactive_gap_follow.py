#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
from numpy.core.fromnumeric import amax, size
from numpy.ma.core import flatten_structured_array

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#Lidar Preprocess variables
detect_angle = 90
past_ranges_t1 = past_ranges_t2 = past_ranges_t3 = past_ranges_t4 = current_ranges_t5 = np.zeros(int(1080 * detect_angle / 360))

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
        global past_ranges_t1, past_ranges_t2, past_ranges_t3, past_ranges_t4, current_ranges_t5
        past_ranges_t1 = past_ranges_t2
        past_ranges_t2 = past_ranges_t3
        past_ranges_t3 = past_ranges_t4
        past_ranges_t4 = current_ranges_t5
        current_ranges_t5 = ranges
        
        # Get weighted rolling mean among 5 frames
        proc_ranges = 0.5 * current_ranges_t5 + 0.25 * past_ranges_t4 + 0.15 * past_ranges_t3 + 0.07 * past_ranges_t2 + 0.03 * past_ranges_t1
        
        # Rejecting high values
        proc_ranges[proc_ranges > 3.0] = 3
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        index_list = []
        max_gap_value = 0
        start = True

        # Retrieve all start indeces and end indeces for the gap
        for i in range(size(free_space_ranges)):
            if free_space_ranges[i] != 0 and start == True:
                index_list.append(i)
                start = False
            elif free_space_ranges[i] == 0 and start == False:
                index_list.append(i-1)
                start = True

        # Add in the last index if end index have not append into the list above
        if (len(index_list) % 2 == 1):
            index_list.append(size(free_space_ranges) - 1)

        # Compute the max gap start and end indeces
        for i in range(int(len(index_list) / 2)):
                gap_value = index_list[2 * i + 1] - index_list[2 * i]
                if gap_value > max_gap_value:
                    max_gap_value = gap_value
                    max_gap_index = [index_list[2 * i], index_list[2 * i +1]]
        return max_gap_index
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        
        return np.argmax(ranges[start_i : end_i])

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = np.asarray(data.ranges)
        # filter unwanted ranges data
        ranges = ranges[int(0.5 * size(ranges) - size(ranges) * detect_angle/ (2 * 360)): int(0.5 * size(ranges) + size(ranges) * detect_angle/ (2 * 360))]
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closest_point = np.argmin(proc_ranges)
        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius = 0.35
        for i in range(size(proc_ranges)):
            if math.sqrt(proc_ranges[i] ** 2 + proc_ranges[closest_point] ** 2 - 2 * proc_ranges[i] * proc_ranges[closest_point] * math.cos(data.angle_increment)) < bubble_radius:
                proc_ranges[i] = 0   

        #Find max length gap 
        start_i = self.find_max_gap(proc_ranges)[0]
        end_i = self.find_max_gap(proc_ranges)[1]

        #Find the best point in the gap
        best_point_index = self.find_best_point(start_i, end_i, ranges) + start_i

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        angle = (best_point_index - len(ranges) / 2) * data.angle_increment
        print(angle)
        if abs(angle) < 0.15:
            drive_msg.drive.steering_angle = 0
        else:
            drive_msg.drive.steering_angle = angle
        if abs(angle) > math.radians(0) and abs(angle) <= math.radians(10):
            drive_msg.drive.speed = 1.5
            # drive_msg.drive.steering_angle_velocity = 0.2
        elif abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            drive_msg.drive.speed = 1.0
            # drive_msg.drive.steering_angle_velocity = 0.3
        else:
            drive_msg.drive.speed = 0.5
            # drive_msg.drive.steering_angle_velocity = 0.5
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
