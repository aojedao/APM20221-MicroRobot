#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.last_error = 0.0
    
    def compute(self, error):
        self.error = error
        self.integral += error
        self.derivative = error - self.last_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.derivative)
        self.last_error = error
        return output

class FollowLine:
    def __init__(self):
        rospy.init_node('follow_line', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pid_controller = PIDController(kp=0.5, ki=0.0, kd=0.0)
        self.target_distance = 0.5
    
    def odom_callback(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        linear_velocity = odom.twist.twist.linear.x
        angular_velocity = odom.twist.twist.angular.z
        
        distance_error = x - self.target_distance
        output = self.pid_controller.compute(distance_error)
        
        twist = Twist()
        twist.linear = Vector3(output, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        follow_line = FollowLine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

