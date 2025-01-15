#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum
from segway_msgs.srv import RosSetChassisEnableCmdSrv as RosSetChassisEnableCmd
from segway_msgs.msg import ChassisModeFb

class State(Enum):
    DISABLED = 0  # solid yellow
    ENABLED = 1  # solid green
    PASSIVE = 2  # solid white (push)
    STOPPED = 3  # solid red
    PAUSED = 4  # no extra visual feedback, solid yellow

class StateMachineNode:
    def __init__(self):
        rospy.init_node('state_machine_node', anonymous=True)

        # Initialize state and other variables
        self.state = State.DISABLED
        self.timeout = 20.0  # Timeout in seconds

        self.twist = Twist()
        self.latest_cmd_vel = Twist()
        self.abs_x = 0.0
        self.abs_z = 0.0

        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_out', Twist, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel_mux', Twist, self.cmd_vel_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.chassis_mode_sub = rospy.Subscriber('/chassis_mode_fb', ChassisModeFb, self.chassis_mode_callback)

        # Service clients
        rospy.wait_for_service('set_chassis_enable')
        self.chassis_enable_client = rospy.ServiceProxy('set_chassis_enable', RosSetChassisEnableCmd)
        rospy.loginfo('Chassis enable service available.')

        # Timer for periodic updates
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)  # 100 Hz

    def chassis_mode_callback(self, msg):
        if self.state == State.PAUSED:
            return

        if msg.chassis_mode == 0:
            self.state = State.DISABLED
            rospy.loginfo('Set chassis_mode to DISABLED')
        elif msg.chassis_mode == 1:
            self.state = State.ENABLED
            rospy.loginfo('Set chassis_mode to ENABLED')
        elif msg.chassis_mode == 2:
            self.state = State.PASSIVE
            rospy.loginfo('Set chassis_mode to PASSIVE')
        elif msg.chassis_mode == 3:
            self.state = State.STOPPED
            rospy.loginfo('Set chassis_mode to STOPPED')

    def enable_chassis(self):
        try:
            self.chassis_enable_client(True)
            self.state = State.ENABLED
            rospy.loginfo('Enabling chassis...')
        except rospy.ServiceException as e:
            rospy.logerr(f'Failed to enable chassis: {e}')

    def pause_chassis(self):
        try:
            self.chassis_enable_client(False)
            self.state = State.PAUSED
            rospy.loginfo('Pausing chassis...')
        except rospy.ServiceException as e:
            rospy.logerr(f'Failed to pause chassis: {e}')

    def disable_chassis(self):
        try:
            self.chassis_enable_client(False)
            self.state = State.DISABLED
            rospy.loginfo('Disabling chassis...')
        except rospy.ServiceException as e:
            rospy.logerr(f'Failed to disable chassis: {e}')

    def joy_callback(self, msg):
        start_button = msg.buttons[7]  # Joystick button 'start'
        select_button = msg.buttons[6]  # Joystick button 'select'

        if start_button == 1:
            rospy.loginfo("State: ENABLED (Button 'start')")
            self.enable_chassis()
            self.timeout = 20

        if select_button == 1:
            rospy.loginfo("State: DISABLED (Button 'select')")
            self.pause_chassis()

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.abs_x = abs(msg.linear.x)
        self.abs_z = abs(msg.angular.z)
        self.timeout = 20.0

    def timer_callback(self, event):
        if self.state in [State.PAUSED, State.STOPPED, State.PASSIVE]:
            return

        if self.state == State.ENABLED:
            if self.timeout <= 0:
                self.state = State.DISABLED
                rospy.loginfo("State: DISABLED (Timeout)")
                self.disable_chassis()
            else:
                self.timeout -= 0.01
                self.cmd_vel_pub.publish(self.latest_cmd_vel)

        if self.state == State.DISABLED and (self.abs_x > 0.002 or self.abs_z > 0.002):
            self.state = State.ENABLED
            rospy.loginfo("State: ENABLED (cmd_vel)")
            self.enable_chassis()

if __name__ == '__main__':
    node = StateMachineNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.disable_chassis()
        rospy.loginfo('Shutting down node...')
