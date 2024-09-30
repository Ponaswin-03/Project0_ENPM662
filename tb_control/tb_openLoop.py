#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import matplotlib.pyplot as plt

class openloopmv(Node):
    def __init__(self):
        super().__init__('mvtb3_openloop')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed_x = 0.3
        self.positions = []  # List to store the position over time
        self.times = []  # List to store the time data
        self.current_position = 0.0  # Initialize position
        self.start_time = time.time()  # Track start time

    def record_position(self):
        """Record the current position and time."""
        current_time = time.time() - self.start_time
        self.positions.append(self.current_position)
        self.times.append(current_time)

    def move(self, mv_duration):
        start = time.time()
        while time.time() - start < mv_duration:
            mv_value = Twist()
            mv_value.linear.x = self.linear_speed_x
            self.vel_pub.publish(mv_value)
            self.get_logger().info("Moving at a speed of %.2f m/s" % self.linear_speed_x)
            
            # Update position (distance = speed * time)
            self.current_position += self.linear_speed_x * 0.1  # Assuming loop rate is 0.1 seconds
            self.record_position()
            
            time.sleep(0.1)  # Loop rate of 100 ms
        
        mv_value.linear.x = 0.0
        self.vel_pub.publish(mv_value)
        self.get_logger().info('Stopped after %.2f seconds' % mv_duration)
        
        # Plot position over time
        self.plot_position_over_time()

    def plot_position_over_time(self):
        """Plot the position of the robot over time."""
        plt.figure()
        plt.plot(self.times, self.positions, label='Position (m)')
        plt.title('Robot Position Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.grid(True)
        plt.legend()
        plt.show()

def main():
    rclpy.init()
    mvtb3_openloop = openloopmv()
    mvtb3_openloop.move(5.0)  # Move for 5 seconds
    rclpy.spin(mvtb3_openloop)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
