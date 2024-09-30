#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotMotion(Node):
    def __init__(self):
        super().__init__('turtlebot_motion')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_speed = 0.3  # Max velocity (m/s)
        self.acceleration = 0.1  # Acceleration (m/s^2)
        self.rate = 0.1  # Time step for each loop (s)

    def accelerate(self, target_speed):
        """ Accelerate to the target speed """
        speed = 0.0
        while speed < target_speed:
            mv_value = Twist()
            speed = min(speed + self.acceleration * self.rate, target_speed)
            mv_value.linear.x = speed
            self.vel_pub.publish(mv_value)
            self.get_logger().info("Accelerating: Speed = %.2f m/s" % speed)
            time.sleep(self.rate)

    def move_at_constant_speed(self, distance):
        """ Move at a constant speed over a specified distance """
        mv_value = Twist()
        mv_value.linear.x = self.max_speed
        start = time.time()
        duration = distance / self.max_speed  # Time to travel the given distance

        while time.time() - start < duration:
            self.vel_pub.publish(mv_value)
            self.get_logger().info("Moving at constant speed: %.2f m/s" % self.max_speed)
            time.sleep(self.rate)

    def decelerate(self):
        """ Decelerate to a complete stop """
        speed = self.max_speed
        while speed > 0:
            mv_value = Twist()
            speed = max(speed - self.acceleration * self.rate, 0)
            mv_value.linear.x = speed
            self.vel_pub.publish(mv_value)
            self.get_logger().info("Decelerating: Speed = %.2f m/s" % speed)
            time.sleep(self.rate)

    def move(self, distance_acceleration, distance_constant):
        # Phase 1: Acceleration
        self.accelerate(self.max_speed)

        # Phase 2: Move at constant speed for distance_constant
        self.move_at_constant_speed(distance_constant)

        # Phase 3: Deceleration
        self.decelerate()
        self.get_logger().info("Motion complete")

def main():
    rclpy.init()
    turtlebot_motion = TurtleBotMotion()

    # Move with acceleration, constant speed, and deceleration
    distance_acceleration = 1.0  # Distance covered while accelerating (m)
    distance_constant = 2.0  # Distance covered at constant speed (m)
    
    turtlebot_motion.move(distance_acceleration, distance_constant)

    rclpy.spin(turtlebot_motion)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
