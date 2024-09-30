#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import matplotlib.pyplot as plt

class TurtleBotMotion(Node):
    def __init__(self):
        super().__init__('turtlebot_motion')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_speed = 0.3
        self.acceleration = 0.1
        self.rate = 0.1
        
        self.positions = []
        self.times = []
        self.current_position = 0.0
        self.start_time = time.time()

    def record_position(self):
        current_time = time.time() - self.start_time
        self.positions.append(self.current_position)
        self.times.append(current_time)
        
    def accelerate(self, target_speed):
        speed = 0.0
        while speed < target_speed:
            mv_value = Twist()
            speed = min(speed + self.acceleration * self.rate, target_speed)
            mv_value.linear.x = speed
            self.vel_pub.publish(mv_value)
            self.get_logger().info("Accelerating at speed of %.2f m/s" % speed)
            time.sleep(self.rate)
            self.current_position += speed * self.rate
            self.record_position()

    def move_at_constant_speed(self, distance):
        mv_value = Twist()
        mv_value.linear.x = self.max_speed
        start = time.time()
        duration = distance / self.max_speed

        while time.time() - start < duration:
            self.vel_pub.publish(mv_value)
            self.get_logger().info("Moving at a constant speed of %.2f m/s" % self.max_speed)
            time.sleep(self.rate)
            self.current_position += self.max_speed * self.rate
            self.record_position()

    def decelerate(self):
        speed = self.max_speed
        while speed > 0:
            mv_value = Twist()
            speed = max(float(speed - self.acceleration * self.rate), 0.0)
            mv_value.linear.x = float(speed)
            self.vel_pub.publish(mv_value)
            self.get_logger().info("Decelerating at speed of %.2f m/s" % speed)
            time.sleep(self.rate)
            self.current_position += speed * self.rate
            self.record_position()


    def move(self, distance_acceleration, distance_constant):
        self.positions.clear()
        self.times.clear()
        self.start_time = time.time()
        self.accelerate(self.max_speed)
        self.move_at_constant_speed(distance_constant)

        self.decelerate()
        self.get_logger().info("Motion complete")
        self.plot_position_over_time()

    def plot_position_over_time(self):
        plt.figure()
        plt.plot(self.times, self.positions, label='Position (m)')
        plt.title('TurtleBot Position Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.grid(True)
        plt.legend()
        plt.show()

def main():
    rclpy.init()
    
    turtlebot_motion = TurtleBotMotion()
    distance_acceleration = 1.0
    distance_constant = 2.0
    turtlebot_motion.move(distance_acceleration, distance_constant)

    rclpy.spin(turtlebot_motion)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
