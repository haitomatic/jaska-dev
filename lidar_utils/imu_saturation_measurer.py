#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import time
import signal
import sys

class IMUSaturationMeasurer(Node):
    def __init__(self):
        super().__init__('imu_saturation_measurer')
        
        # Subscribe to IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/unilidar/imu',
            self.imu_callback,
            10
        )
        
        # Data storage
        self.linear_accelerations = []
        self.angular_velocities = []
        
        # Statistics
        self.message_count = 0
        self.start_time = time.time()
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info("IMU Saturation Measurer started!")
        self.get_logger().info("Move the LiDAR aggressively in all directions...")
        self.get_logger().info("Press Ctrl+C when done to see results")
        
    def imu_callback(self, msg):
        # Extract linear acceleration (x, y, z)
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y  
        acc_z = msg.linear_acceleration.z
        
        # Calculate magnitude
        acc_magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        self.linear_accelerations.append(acc_magnitude)
        
        # Extract angular velocity (x, y, z)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        
        # Calculate magnitude
        gyro_magnitude = np.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
        self.angular_velocities.append(gyro_magnitude)
        
        self.message_count += 1
        
        # Print progress every 100 messages
        if self.message_count % 100 == 0:
            elapsed = time.time() - self.start_time
            rate = self.message_count / elapsed
            self.get_logger().info(f"Collected {self.message_count} samples "
                                 f"({rate:.1f} Hz) - "
                                 f"Current acc: {acc_magnitude:.2f}, gyro: {gyro_magnitude:.2f}")
    
    def analyze_data(self):
        if len(self.linear_accelerations) == 0:
            self.get_logger().error("No IMU data collected!")
            return
            
        acc_array = np.array(self.linear_accelerations)
        gyro_array = np.array(self.angular_velocities)
        
        elapsed_time = time.time() - self.start_time
        
        print("\n" + "="*60)
        print("IMU SATURATION ANALYSIS RESULTS")
        print("="*60)
        
        print(f"\nData Collection Summary:")
        print(f"  Total samples: {len(acc_array)}")
        print(f"  Collection time: {elapsed_time:.1f} seconds")
        print(f"  Average rate: {len(acc_array)/elapsed_time:.1f} Hz")
        
        print(f"\nLinear Acceleration Analysis (m/s²):")
        print(f"  Min: {np.min(acc_array):.3f}")
        print(f"  Max: {np.max(acc_array):.3f}")
        print(f"  Mean: {np.mean(acc_array):.3f}")
        print(f"  95th percentile: {np.percentile(acc_array, 95):.3f}")
        print(f"  99th percentile: {np.percentile(acc_array, 99):.3f}")
        
        print(f"\nAngular Velocity Analysis (rad/s):")
        print(f"  Min: {np.min(gyro_array):.3f}")
        print(f"  Max: {np.max(gyro_array):.3f}")
        print(f"  Mean: {np.mean(gyro_array):.3f}")
        print(f"  95th percentile: {np.percentile(gyro_array, 95):.3f}")
        print(f"  99th percentile: {np.percentile(gyro_array, 99):.3f}")
        
        print(f"\n" + "="*60)
        print("RECOMMENDED VALUES FOR unilidar_l2.yaml:")
        print("="*60)
        
        # Use 95th percentile as saturation values (conservative approach)
        recommended_satu_acc = np.percentile(acc_array, 95)
        recommended_satu_gyro = np.percentile(gyro_array, 95)
        
        print(f"\nmapping:")
        print(f"    satu_acc: {recommended_satu_acc:.1f}   # 95th percentile of observed accelerations")
        print(f"    satu_gyro: {recommended_satu_gyro:.1f}  # 95th percentile of observed angular velocities")
        print(f"    acc_norm: 9.81        # Keep as is (gravity constant)")
        
        # Alternative conservative values (99th percentile)
        alt_satu_acc = np.percentile(acc_array, 99)
        alt_satu_gyro = np.percentile(gyro_array, 99)
        
        print(f"\n# Alternative (more conservative - 99th percentile):")
        print(f"# satu_acc: {alt_satu_acc:.1f}")
        print(f"# satu_gyro: {alt_satu_gyro:.1f}")
        
        print(f"\n" + "="*60)
        print("NOTES:")
        print("- Move the LiDAR more aggressively if max values seem low")
        print("- The 95th percentile values should work well for most cases")
        print("- Use 99th percentile values if you expect very aggressive motion")
        print("="*60)
    
    def signal_handler(self, signum, frame):
        print("\n\nShutting down and analyzing data...")
        self.analyze_data()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    
    measurer = IMUSaturationMeasurer()
    
    try:
        rclpy.spin(measurer)
    except KeyboardInterrupt:
        pass
    finally:
        measurer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()