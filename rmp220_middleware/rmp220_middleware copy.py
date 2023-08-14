#!/usr/bin/env python3

import rclpy
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Joy
from segway_msgs.srv import RosSetChassisEnableCmd

class State(Enum):
    DISABLED = 0
    ENABLED = 1

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # Initialize state and other variables
        self.state = State.DISABLED
        self.timeout = 2.0  # Timeout in seconds

        # Create publishers, subscribers, timers, and service clients here
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_out', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_mux', self.cmd_vel_callback, 10)
        #self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create service clients for chassis enable and disable
        self.chassis_enable_client = self.create_client(RosSetChassisEnableCmd, 'set_chassis_enable')
        while not self.chassis_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting for chassis enable service...')
        self.chassis_disable_client = self.create_client(RosSetChassisEnableCmd, 'set_chassis_enable')
        while not self.chassis_disable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting for chassis disable service...')

    def joy_callback(self, msg):
        # Implement logic to detect joystick button presses (start/select) and update state
        # ...
        if msg.buttons[7] == 1:  # Joystick button 'start'
            self.state = State.ENABLED
            self.get_logger().info("State: ENABLED (Button 'start')")
            self.enable_chassis()
        if msg.buttons[6] == 1:  # Joystick button 'select'
            self.state = State.DISABLED
            self.get_logger().info("State: DISABLED (Button 'select')")
            self.disable_chassis()

    def enable_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = True
        self.chassis_enable_client.call_async(req)

    def disable_chassis(self):
        req = RosSetChassisEnableCmd.Request()
        req.ros_set_chassis_enable_cmd = False
        self.chassis_disable_client.call_async(req)

    def cmd_vel_callback(self, msg):
        # Update state to ENABLED upon receiving a command on /cmd_vel_mux
        # ...
        if self.state == State.ENABLED:
            self.cmd_vel_pub.publish(msg)
            self.timeout = 2.0  # Reset timeout when receiving commands

    def timer_callback(self):
            # Republish the cmd_vel_mux command to cmd_vel_out topic
            # ...

            # Reset the timeout counter
            # ...

            # Check if the timeout has been exceeded, and if so, switch to DISABLED
            # ...

        if self.state == State.ENABLED:
            if self.timeout <= 0:
                self.state = State.DISABLED
                self.get_logger().info("State: DISABLED (Timeout)")
                self.disable_chassis()
            else:
                self.timeout -= 0.1

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
