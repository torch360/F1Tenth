#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
 
class AutoBrakeNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("auto_brake") # MODIFY NAME
        self.laser_subscriber_ = self.create_subscription(LaserScan, "scan", self.find_ittc, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, "ego_racecar/odom", self.set_odom, 10)
        self.ackermann_publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.velocity_= 0.001
        self.theta_ = np.arange(720) * 0.004351851996034384
    
    def set_odom(self, msg):
        self.velocity_ = msg.twist.twist.linear.x
        
    
    def find_ittc(self, msg):
        #restricts scan to what is happening in front of the car from 0 to pi radians
        new_scan = np.array(msg.ranges[180:900])
        rdot = self.velocity_*np.cos(self.theta_)
        
        #make sure positive and non-zero denominator
        
        denom = np.where(-1*rdot<=0.0000001, 0.00001, -1*rdot)
        
        #r/rdot
        ittc = new_scan/denom
        clean_ittc = np.nan_to_num(ittc, posinf=1000, neginf=100)
        
        #take the smallest value to find the closest object
        threat = np.min(clean_ittc)
        
        
        ##ATTEMPTED IMPLEMENTATION OF STAGED AEB
        #print(threat)
        # if(threat < 2 and threat > 1.7 and self.velocity_ > 1.5):
        #     caution = AckermannDriveStamped()
        #     caution.drive.speed = 1.5
        #     print("slow")
        #     self.ackermann_publisher_.publish(caution)
        # elif(threat< 1.7 and threat > 1.0 and self.velocity_ > .5):
        #     warning = AckermannDriveStamped()
        #     warning.drive.speed = .5
        #     self.ackermann_publisher_.publish(warning)
        #     print("HALTHALT")
    
        if threat< .65:
            STOP = AckermannDriveStamped()
            STOP.drive.speed = 0.0
            self.ackermann_publisher_.publish(STOP)
            print("HALT")
        
        
        
        
 
 
def main(args=None):
    rclpy.init(args=args)
    node = AutoBrakeNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()