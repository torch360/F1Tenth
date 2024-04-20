from asyncio import proactor_events
from multiprocessing.connection import wait
from xml.dom.pulldom import PROCESSING_INSTRUCTION
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.lidar_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.bubble_size = 10
        self.average_window = 4
        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive
    
    def moving_average(a, n):
        """Return the mean of [a-n,a+n] in an array"""
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)
        proc_ranges = np.where(proc_ranges > 3, proc_ranges, 3)
        proc_ranges = np.where(proc_ranges < .4, proc_ranges, 0)
        proc_ranges = moving_average(proc_ranges, self.average_window)
        proc_ranges = proc_ranges[180-self.bubble_size : 900 - self.bubble_size] # because the array is 4 less on either side, offset by 10 to get entire 180
        return proc_ranges



    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        start_index = 0
        end_index = 0
        ctr = 0
        curr_index = 0
        for item in free_space_ranges:
            if item > 0:
                ctr += 1
                if ctr > end_index-start_index: #if new max reset start and end indices
                    start_index = curr_index - ctr #start index is current - how far away the non-zero sub-array started
                    end_index = curr_index # end is current
            else:
                ctr = 0

            curr_index += 1
            
        return start_index, end_index
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        waypoint = np.floor(start_i + end_i / 2)
        waypoint = waypoint - (540 - self.bubble_size)                  # recenter to middle of array             
        angle = waypoint * 0.004351851996034384         #multiply by angle increment
        return angle

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        
        # TODO:
        #Find closest point to LiDAR
        minimum = np.argmin(proc_ranges)
        #Eliminate all points inside 'bubble' (set them to zero) 
        proc_ranges[minimum - self.bubble_size : minimum + self.bubble_size] = [0] * 2 * self.bubble_size
        #Find max length gap 
        start, end = self.find_max_gap(proc_ranges)
        #Find the best point in the gap 
        angle = self.find_best_point(start, end, proc_ranges)
        print(angle, self.time_)
        if 0 < np.abs(angle) * 180/np.pi < 10:
            velocity = 1.5
            print("NICE")
        elif 10 < np.abs(angle) * 180/np.pi < 20:
            velocity = 1.0
        else:
            velocity = 0.5

        #Publish Drive message
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = angle
        msg.drive.speed = velocity
        self.drive_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()