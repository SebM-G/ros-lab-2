# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import math

SEARCH  = 0
AIM     = 1
TRAVEL  = 2
desired_distance = 2.5

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.publisher_ = self.create_publisher(Twist, 'diff_drive/cmd_vel', 10)

        self.subscription = self.create_subscription(
            LaserScan,              
            '/diff_drive/scan',     
            self.callback,          
            10                     
        )

        self.state = TRAVEL
        self.prev_range = float('inf')
        self.corner_type = None  # 'concave' or 'convex'
        self.forward_speed = 5.0
        self.turn_speed = 0.4
        self.turn_mod = -1.0
        self.goal = desired_distance
        self.Kp = 5
        self.filtered_range = None
        self.cutoff_freq = .1
        self.prev_time = self.get_clock().now()

    def callback(self, msg):

        """---------------------------- Low Pass Filter ----------------------------"""
        # Time delta
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        raw_range = msg.ranges[0]

        # Beta Calculation
        fc = self.cutoff_freq
        beta = (2 * math.pi * fc * dt) / (2 * math.pi * fc * dt + 1)

        # Filter Calculation
        if self.filtered_range is None:
            self.filtered_range = raw_range 
        else:
            self.filtered_range = beta * self.filtered_range + (1 - beta) * raw_range
        """-----------------------------------------------------------------------"""

        current_range = self.filtered_range

        self.get_logger().info(f"Front range: {raw_range:.2f}m Filtered Range {current_range:.2f} m")

        cmd = Twist()

        if self.state == TRAVEL:
            """
            Meant to move along a straight line until
            the next turn is ready to be made.
            """
            if self.goal is None:
                self.goal = self.desired_distance

            if self.goal < current_range:
                error = current_range - self.goal
                cmd.linear.x = self.forward_speed * self.Kp * error
            else:
                self.state = SEARCH
                self.goal = None
                self.get_logger().info("Entering Search")


        elif self.state == SEARCH:
            """
            Meant to search for the wall it's meant to 
            move towards. Does so by searching for a
            "spike" in the distance over a turn.

            Also capable of differentiating between
            convex and concave turns.
            """

            range_delta = current_range - self.prev_range

            cmd.angular.z = self.turn_speed * self.turn_mod

            if current_range < desired_distance:
                # Helps navigate tight spaces by backing up
                cmd.linear.x = -self.forward_speed * (desired_distance - current_range)


            if self.corner_type == 'convex':
                # Search adjustment necessary for accomidating convex corners
                if current_range < desired_distance:
                    self.corner_type = None
                    self.turn_mod = -self.turn_mod

            elif range_delta > 5.0:
                # Detected convex corner (sudden jump)
                self.get_logger().info("Convex corner detected.")
                self.corner_type = 'convex'
                self.goal = math.sqrt(self.prev_range**2 + desired_distance**2)
                self.state = AIM
                self.get_logger().info(f"Entering AIM, Shooting for distance {self.goal:.2f}")

            elif range_delta < 0:
                # Detected concave corner (peak and fall)
                self.get_logger().info("Concave corner detected.")
                self.corner_type = 'concave'
                self.goal = desired_distance  # stop short
                self.state = AIM
                self.get_logger().info("Entering AIM")

        elif self.state == AIM:
            """
            This state is meant to line us up with a wall.
            It assume's we're pointed at the wall we need 
            to move to, and that we're turning in the
            right direction to get there.

            It is supposed to stop when pointing at the 
            nearest point of the opposing wall.
            """

            if current_range < desired_distance:
                # Keep turning if we're too close
                self.state = SEARCH
                self.goal = None
                self.get_logger().info("Entering Search")

            else: 
                range_tolerance = 0.001
                range_delta = self.prev_range - current_range

                if range_delta < 0:
                    # Turn if we've gone too far
                    self.turn_mod = -self.turn_mod
                
                cmd.angular.z = self.turn_speed * self.turn_mod * range_delta * self.Kp

                if abs(range_delta) < range_tolerance:
                    # If we're happy with the angle.
                    if self.corner_type == 'convex':
                        self.goal = current_range - self.goal
                        self.turn_mod = 1.0
                    else: 
                        self.turn_mod = -1.0

                    self.state = TRAVEL
                    self.get_logger().info("Entering TRAVEL")


        self.prev_range = current_range
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()