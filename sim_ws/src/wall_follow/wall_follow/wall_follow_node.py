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
        # Ku is 4 ish
        # Tu is .25ms 
        self.kp = .9
        self.kd = self.kp*.2/32
        self.ki = self.kp/10


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
        index = int(np.floor(900 - angle/0.004351851996034384))
        a = range_data[index]  #45 deg angle
        b = range_data[900]  #parallel to rear axle
        if(np.isfinite(a) and np.isfinite(b)):
            alpha = np.arctan((a*np.cos(angle)-b)/(a*np.sin(angle)))
            dist = b*np.cos(alpha)
            self.alpha_ = alpha
        else:
            dist = -1
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
        distance = self.get_range(range_data, 45 * np.pi/180)
        if distance == -1.:
            return False
        else:
            error = dist-(distance + 0.88*np.sin(self.alpha_))
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
            self.integral += error * (self.time_ - self.prevtime_)
            derivative = (error - self.prev_error) / (self.time_ - self.prevtime_)
        elif self.prevtime_ is None:
            return


        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        angle = -(self.kp * error + self.ki * self.integral + self.kd * derivative)
        print(self.kp * error, self.ki * self.integral, self.kd * derivative)
        
        # if angle > .4:
        #     angle = .4
        # elif angle<-.4:
        #     angle=-.4
        print(angle, self.time_)
        if 0 < np.abs(angle) * 180/np.pi < 10:
            velocity = 1.5
            print("NICE")
        elif 10 < np.abs(angle) * 180/np.pi < 20:
            velocity = 1.0
        else:
            velocity = 0.5
            

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.publisher_.publish(drive_msg)
        

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.time_ = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        error = self.get_error(msg.ranges, 1) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actiuate the car with PID
        self.prevtime_ = self.time_





def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    print("running")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()