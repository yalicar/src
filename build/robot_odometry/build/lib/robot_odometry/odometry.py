# publish odometry message to /odom topic from /base_link suscriber topic /cmd_vel
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# import quaternion
# import numpy as np
# import math
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'wheel/odometry', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.position.y = 0.0
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.pose.pose.orientation.w = 1.0
        self.odom.twist.twist.linear.x = 0.0
        self.odom.twist.twist.linear.y = 0.0
        self.odom.twist.twist.linear.z = 0.0
        self.odom.twist.twist.angular.x = 0.0
        self.odom.twist.twist.angular.y = 0.0
        self.odom.twist.twist.angular.z = 0.0
        self.get_logger().info('Odometry publisher has been started')
        
    def listener_callback(self, msg):
        self.odom.twist.twist.linear.x = msg.linear.x
        self.odom.twist.twist.angular.z = msg.angular.z
        self.get_logger().info('I heard: "%f"' % msg.linear.x)
        self.get_logger().info('I heard: "%f"' % msg.angular.z)
        
    def timer_callback(self):
        self.odom.header.stamp = self.get_clock().now().to_msg()
        # calculate odometry in a typical way given the velocities of the robot
        dt = 0.1
        delta_x = 0
        delta_y = 0
        delta_th = (self.odom.twist.twist.angular.z * dt)
        self.odom.pose.pose.position.x += 0.0
        self.odom.pose.pose.position.y += 0.0
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.pose.pose.orientation.w = 1.0        

        self.publisher_.publish(self.odom)
        self.get_logger().info('Publishing: "%f"' % self.odom.twist.twist.linear.x)
        self.get_logger().info('Publishing: "%f"' % self.odom.twist.twist.angular.z)

def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    rclpy.spin(odometry_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    



