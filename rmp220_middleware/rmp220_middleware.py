#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum
from segway_msgs.srv import RosSetChassisEnableCmd
from segway_msgs.msg import ChassisModeFb


import atexit
import signal
import sys

class State(Enum): # is this best-practice?
    DISABLED = 0 # solid yellow
    ENABLED = 1 # solid green
    PASSIVE = 2 # solid white (push)
    STOPPED = 3 # solid red
    PAUSED = 4 # no extra visual feedback, solid yellow

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
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Create twist class for publishing velocities
        self.twist = Twist()

        self.latest_cmd_vel = Twist()
        self.abs_x = 0.0
        self.abs_z = 0.0

        # Create service clients for chassis enable and disable
        self.chassis_enable_client = self.create_client(RosSetChassisEnableCmd, 'set_chassis_enable')
        while not self.chassis_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting for chassis enable service...')
        self.get_logger().info('Chassis enable service available.')

        # Create a subscriber for the chassis status topic
        self.chassis_mode_sub = self.create_subscription(
            ChassisModeFb,
            '/chassis_mode_fb',
            self.chassis_mode_callback,
            10
        )

        self.chassis_mode = State.DISABLED

    def chassis_mode_callback(self, msg):
        # Handle the incoming chassis status message, will update every second
        # Need to figure out the type
        self.get_logger().info('Got chassis_mode from chassis_mode_fb topic')
        self.get_logger().info(str(msg.chassis_mode))

        if msg.chassis_mode == 0:
            self.chassis_mode = State.DISABLED
            self.state = State.DISABLED
            self.get_logger().info('Set chassis_mode to ' + str(self.state.value))
        if msg.chassis_mode == 1:  # Assuming 1 represents enabled and 0 represents disabled
            self.chassis_mode = State.ENABLED
            self.state = State.ENABLED
            self.get_logger().info('Set chassis_mode to ' + str(self.state.value))
        if msg.chassis_mode == 2:
            self.chassis_mode = State.PASSIVE
            self.state = State.PASSIVE
            self.get_logger().info('Set chassis_mode to ' + str(self.state.value))
        if msg.chassis_mode == 3:
            self.chassis_mode = State.STOPPED
            self.state = State.STOPPED
            self.get_logger().info('Set chassis_mode to ' + str(self.state.value))
        
    def get_chassis_mode(self):
        return self.chassis_mode

    def enable_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = True
        self.chassis_enable_client.call_async(req)
        self.state = State.ENABLED
        self.get_logger().info('Enabling chassis...')

    def pause_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = False
        self.chassis_enable_client.call_async(req)
        self.state = State.PAUSED
        self.get_logger().info('Pausing chassis...')

    def disable_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = False
        self.chassis_enable_client.call_async(req)
        self.state = State.DISABLED
        self.get_logger().info('Disabling chassis...')

    def joy_callback(self, msg):
        start_button = msg.buttons[7] # Joystick button 'start'
        select_button = msg.buttons[6] # Joystick button 'select'

        if start_button == 1:
            self.state = State.ENABLED
            self.get_logger().info("State: ENABLED (Button 'start')")
            self.enable_chassis()
        if select_button == 1:
            self.state = State.PAUSED
            self.get_logger().info("State: DISABLED (Button 'select')")
            self.disable_chassis()

    def cmd_vel_callback(self, msg):
        # This method shall only update the latest_cmd_vel attribute so it can be republished by the timer_callback with 100 HZ. Should have a look at performance though.
        self.latest_cmd_vel = msg

        # Will also derive the absolute values of the linear and angular velocity
        self.abs_x = abs(msg.linear.x)
        self.abs_z = abs(msg.angular.z)

        # Reset timeout when receiving commands
        self.timeout = 20.0

    def timer_callback(self):
        # if self.chassis_mode == State.DISABLED or self.chassis_mode == State.STOPPED or self.chassis_mode == State.PASSIVE:
        #     return # Do nothing if chassis is disabled, stopped or passive --> should save processing power
        if self.state == State.PAUSED:
            return # Do nothing if state is paused --> should save processing power
        if self.state == State.ENABLED:
            if self.timeout <= 0:
                self.state = State.DISABLED
                self.get_logger().info("State: DISABLED (Timeout)")
                self.disable_chassis()
            else:
                self.timeout -= 0.01
                self.cmd_vel_pub.publish(self.latest_cmd_vel)
        #if self.state == State.PAUSED and (self.abs_x > 0.1 or self.abs_z > 0.1): # This is a hack to enable the chassis when receiving commands e.g. from Nav2
        if self.state == State.DISABLED and (self.abs_x > 0.1 or self.abs_z > 0.1): # This is a hack to enable the chassis when receiving commands e.g. from Nav2
            self.state = State.ENABLED
            self.get_logger().info("State: ENABLED (cmd_vel)")
            self.enable_chassis()
        # if self.state == State.PAUSED and (self.abs_x < 0.1 and self.abs_z < 0.1): # Is this even necessary?
        #     self.cmd_vel_pub.publish(self.twist)

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

if __name__ == '__main__':
    main()
