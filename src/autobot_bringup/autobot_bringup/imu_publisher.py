import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from .lib import IMU
import math

class IMUPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        self.publisher = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.imu = IMU()
        
        self.gyro_samples = []
        self.calibration_done = False
        self.bias = [0.0, 0.0, 0.0]
        self.calibration_time = 2.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
    def read_imu(self):
        accel = self.imu.get_accel()
        gyro = self.imu.get_gyro()
        
        return accel, gyro

    def timer_callback(self):
        accel, gyro = self.read_imu()
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        if not self.calibration_done:
            self.gyro_samples.append(gyro)
            if t >= self.calibration_time:
                self.bias = [sum(x)/len(self.gyro_samples) for x in zip(*self.gyro_samples)]
                self.calibration_done = True
                self.get_logger().info(f"Gyro bias calculated: {self.bias}")
            return
        
        accel, gyro = self.read_imu()
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        imu_msg.angular_velocity.x = math.radians(gyro[0] - self.bias[0])
        imu_msg.angular_velocity.y = math.radians(gyro[1] - self.bias[1])
        imu_msg.angular_velocity.z = math.radians(gyro[2] - self.bias[2])
        
        imu_msg.linear_acceleration.x = accel[0] * 9.80665
        imu_msg.linear_acceleration.y = accel[1] * 9.80665
        imu_msg.linear_acceleration.z = accel[2] * 9.80665
        
        imu_msg.orientation_covariance = [9999999, 0, 0,
                                          0, 9999999, 0,
                                          0, 0, 0.01]
        imu_msg.angular_velocity_covariance = [0.02, 0, 0,
                                               0, 0.02, 0,
                                               0, 0, 0.01]
        imu_msg.linear_acceleration_covariance = [0.04, 0, 0,
                                                  0, 0.04, 0,
                                                  0, 0, 0.04]
        
        self.publisher.publish(imu_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()