import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.subscriber_ = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        # TODO: set PID gains
        self.kp = 1
        self.kd = 1
        self.ki = 1


        # TODO: store history
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # TODO: store any necessary values you think you'll need
        self.alpha_ = 1
        self.prevtime_ = None
        self.time_ = None

    def get_range(self, range_data, angle) -> float:
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        index = np.floor(900 - angle/0.004351851996034384)
        a = range_data(900-index)  #45 deg angle
        b = range_data(900)  #parallel to rear axle
        if(np.isfinite(a) and np.isfinite(b)):
            alpha = np.arctan((a*np.cos(angle)-b)/(a*np.sin(angle)))
            dist = b*np.cos(alpha)
            self.alpha_ = alpha
        else:
            dist = -1.

        return dist

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter-clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        distance = self.get_range(range_data, 0.785398)
        if distance == -1.:
            return False
        else:
            error = dist-(distance + 0.2*np.sin(self.alpha_))
            return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        if self.prevtime_ is not None:
            self.integral += self.error * (self.time_ - self.prevtime_)
            derivative = (self.error - self.prev_error) / (self.time_ - self.prevtime_)
        else:
            self.integral = 0
            derivative = 0


        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        angle = self.kp * error + self.ki * self.integral + self.kd * derivative

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.time_ = msg.header.seq.sec + msg.header.seq.nsec * 1e-9
        error = self.get_error(msg.ranges, 1) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actiuate the car with PID
        self.prevtime_ = self.time_





def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()