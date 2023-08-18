# main.py

import rclpy
from rmp220_middleware import StateMachineNode

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
