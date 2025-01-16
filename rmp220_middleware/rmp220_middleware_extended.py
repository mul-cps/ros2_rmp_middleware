import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from segway_msgs.srv import ros_set_chassis_enable_cmd as RosSetChassisEnableCmdSrv
from segway_msgs.msg import ChassisModeFb  # Ensure this is the correct message type
from enum import Enum

# Define State enum for states management
class State(Enum):
    DISABLED = 0
    ENABLED = 1
    PAUSED = 2
    STOPPED = 3
    PASSIVE = 4

class JoystickServiceCaller:
    def __init__(self):
        rospy.init_node('joystick_service_caller')

        # Map buttons for select and start
        self.select_button = 6  # Select button index
        self.start_button = 7   # Start button index

        # Initialize service proxy
        self.service_proxy = rospy.ServiceProxy('/ros_set_chassis_enable_cmd_srv', RosSetChassisEnableCmdSrv)

        # Subscribe to the joystick topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        rospy.loginfo("Joystick Service Caller Node Initialized")

    def joy_callback(self, msg):
        try:
            if msg.buttons[self.select_button] == 1:  # Select button pressed
                self.call_service(False)
            elif msg.buttons[self.start_button] == 1:  # Start button pressed
                self.call_service(True)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except IndexError as e:
            rospy.logerr(f"Invalid button index: {e}")

    def call_service(self, enable):
        rospy.wait_for_service('/ros_set_chassis_enable_cmd_srv')
        response = self.service_proxy(ros_set_chassis_enable_cmd=enable)
        rospy.loginfo(f"Service called with enable={enable}, response: {response}")

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

        # Service client initialization with a check
        self.chassis_enable_client = None  # Ensure it's initialized to None
        try:
            rospy.wait_for_service('set_chassis_enable', timeout=5)
            self.chassis_enable_client = rospy.ServiceProxy('set_chassis_enable', RosSetChassisEnableCmdSrv)
            rospy.loginfo('Chassis enable service available.')
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to wait for 'set_chassis_enable' service: {e}")

        # Timer for periodic updates
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)  # 100 Hz

        # Initialize JoystickServiceCaller
        self.joystick_service_caller = JoystickServiceCaller()

    def chassis_mode_callback(self, msg):
        """
        Callback function for handling the chassis mode feedback.
        Sets the state of the robot based on the chassis mode.
        """
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
        if self.chassis_enable_client:
            try:
                self.chassis_enable_client(True)
                self.state = State.ENABLED
                rospy.loginfo('Enabling chassis...')
            except rospy.ServiceException as e:
                rospy.logerr(f'Failed to enable chassis: {e}')
        else:
            rospy.logerr('Chassis enable client not initialized!')

    def pause_chassis(self):
        if self.chassis_enable_client:
            try:
                self.chassis_enable_client(False)
                self.state = State.PAUSED
                rospy.loginfo('Pausing chassis...')
            except rospy.ServiceException as e:
                rospy.logerr(f'Failed to pause chassis: {e}')
        else:
            rospy.logerr('Chassis enable client not initialized!')

    def disable_chassis(self):
        if self.chassis_enable_client:
            try:
                self.chassis_enable_client(False)
                self.state = State.DISABLED
                rospy.loginfo('Disabling chassis...')
            except rospy.ServiceException as e:
                rospy.logerr(f'Failed to disable chassis: {e}')
        else:
            rospy.logerr('Chassis enable client not initialized!')

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
