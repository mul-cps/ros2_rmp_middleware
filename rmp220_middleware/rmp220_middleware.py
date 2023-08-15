#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum
from segway_msgs.srv import RosSetChassisEnableCmd


import atexit
import signal
import sys

class State(Enum):
    DISABLED = 0
    ENABLED = 1

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # Initialize state and other variables
        self.state = State.DISABLED
        self.timeout = 20.0  # Timeout in seconds
        #self.limit = 0.5  # Limit for linear and angular velocity

        # Create publishers, subscribers, timers, and service clients
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_out', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_mux', self.cmd_vel_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        #self.timer = self.create_timer(0.1, self.cmd_vel_callback)

        # Create twist class for publishing velocities
        self.twist = Twist()

        self.latest_cmd_vel = Twist()

        # Create service clients for chassis enable and disable
        self.chassis_enable_client = self.create_client(RosSetChassisEnableCmd, 'set_chassis_enable')
        while not self.chassis_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting for chassis enable service...')
        self.get_logger().info('Chassis enable service available.')
        
        # self.chassis_disable_client = self.create_client(RosSetChassisEnableCmd, 'set_chassis_disable')
        # while not self.chassis_disable_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting for chassis disable service...')
        # self.get_logger().info('Chassis disable service available.')

    def enable_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = True
        self.chassis_enable_client.call_async(req)
        self.get_logger().info('Enabling chassis...')

    def disable_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = False
        self.chassis_enable_client.call_async(req)
        self.get_logger().info('Disabling chassis...')

    def joy_callback(self, msg):
        if self.state == State.DISABLED and self.cmd_vel_sub is not None:
            self.state = State.ENABLED
            self.get_logger().info("State: ENABLED (Joystick)")
            self.enable_chassis()
        if self.state == State.DISABLED and msg.buttons[7] == 1:  # Joystick button 'start'
            self.state = State.ENABLED
            self.get_logger().info("State: ENABLED (Button 'start')")
            self.enable_chassis()
        if self.state == State.ENABLED and msg.buttons[6] == 1:  # Joystick button 'select'
            self.state = State.DISABLED
            self.get_logger().info("State: DISABLED (Button 'select')")
            self.disable_chassis()

    def cmd_vel_callback(self, msg):
        # if self.state == State.ENABLED and (abs(msg.linear.x) >= 0.03 or abs(msg.angular.z) >= 0.03):
        #     self.cmd_vel_pub.publish(msg)
        #     self.timeout = 20.0  # Reset timeout when receiving commands
        #     if msg.linear.x == 0.0 and msg.angular.z == 0.0:
        #         # No data received on cmd_vel_mux, publish zeros
        #         twist_msg = Twist()
        #         self.cmd_vel_pub.publish(twist_msg)
        #     else:
        #         # Forward received data to cmd_vel_out
        #         self.cmd_vel_pub.publish(msg)
        #     self.timeout = 10.0  # Reset timeout when receiving commands

        #     # Forward received data to cmd_vel_out
        #     self.cmd_vel_pub.publish(msg)
        #     self.timeout = 10.0  # Reset timeout when receiving commands

        # if abs(msg.linear.x) > 0.03 or abs(msg.angular.z) > 0.03:
        #     self.latest_cmd_vel = msg
        # else:
        #      self.latest_cmd_vel = Twist()

        # This method shall only update the latest_cmd_vel attribute so it can be republished by the timer_callback with 100 HZ. Should have a look at performance though.
        self.latest_cmd_vel = msg
        # TODO: Add setting chassis state to enabled automatically upon receiving commands (with filter maybe)

    def timer_callback(self):
        #self.cmd_vel_pub.publish(self.twist)
        #self.cmd_vel_callback(self.cmd_vel_sub)
        if self.state == State.ENABLED:
            if self.timeout <= 0:
                self.state = State.DISABLED
                self.get_logger().info("State: DISABLED (Timeout)")
                self.disable_chassis()
            else:
                self.timeout -= 0.1
                self.cmd_vel_pub.publish(self.latest_cmd_vel)
        if self.state == State.DISABLED:
            self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disable_chassis()
        node.destroy_node()
        rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = StateMachineNode()

#     # Register a signal handler for proper cleanup
#     def cleanup_handler(signum, frame):
#         node.disable_chassis()
#         rclpy.shutdown()
#         sys.exit(0)

#     signal.signal(signal.SIGINT, cleanup_handler)
#     atexit.register(cleanup_handler)  # Ensure cleanup on normal exit as well

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         pass

if __name__ == '__main__':
    main()
