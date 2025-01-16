import rospy
from sensor_msgs.msg import Joy
from segway_msgs.srv import ros_set_chassis_enable_cmd as RosSetChassisEnableCmdSrv  # Replace with the actual service type

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

if __name__ == '__main__':
    try:
        node = JoystickServiceCaller()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Joystick Service Caller Node Shutting Down")
