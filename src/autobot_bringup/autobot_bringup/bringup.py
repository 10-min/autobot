import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading, time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
import math

from .lib import PID, Motor

class AutobotBringup(Node):
    def __init__(self):
        super().__init__('autobot_bringup')
        self.motor = Motor()
        
        self.wheel_base = 0.19
        self.wheel_radius = 0.0325  
        self.base_pwm = 10

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.motor.start()
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.left_last, self.right_last = self.motor.get_encoder_counts()
        self.last_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.03, self.update_odom)
        
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z / 5
        
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z
        
        set_l = self.custom_map(left_speed)
        set_r = self.custom_map(right_speed)
        
        self.motor.set_l_motor(set_l)
        self.motor.set_r_motor(set_r)
        
    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Motor에서 엔코더 읽기
        left_count, right_count = self.motor.get_encoder_counts()

        # Δtheta 계산
        delta_left = left_count - self.left_last
        delta_right = right_count - self.right_last
        self.left_last = left_count
        self.right_last = right_count

        wheel_radius = 0.0325
        wheel_sep = 0.19
        encoder_res = 11 * 46.8

        d_left = (2*math.pi*wheel_radius*delta_left)/encoder_res
        d_right = (2*math.pi*wheel_radius*delta_right)/encoder_res

        d_center = (d_left + d_right)/2
        d_theta = (d_right - d_left)/wheel_sep

        self.x += d_center * math.cos(self.th + d_theta/2)
        self.y += d_center * math.sin(self.th + d_theta/2)
        self.th += d_theta

        # Odometry 메시지
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0,0,self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = d_center/dt
        odom.twist.twist.angular.z = d_theta/dt

        self.odom_pub.publish(odom)

        # TF 브로드캐스트
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        #self.tf_broadcaster.sendTransform(t)
            
    def custom_map(self, value):
        if value == 0:
            return 0
        if value > 0:
            result = 25 + ((value * 20) / 0.5)
        else:
            result = -25 + ((value * 20) / 0.5)
        return result
def main(args=None):
    rclpy.init(args=args)
    node = AutobotBringup()
    rclpy.spin(node)
    node.motor.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()