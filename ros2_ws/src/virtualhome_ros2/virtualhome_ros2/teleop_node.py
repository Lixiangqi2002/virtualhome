import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import tty
import select

msg = """
Control The VirtualHome Agent using Continuous Twist Velocities!
---------------------------
Moving around directional velocities:
        w
   a    s    d

w/s : Linear.x +/- (Forward / Backward)
a/d : Angular.z +/- (Turn Left / Right)

Space : STOP (Zero velocity)
CTRL-C to quit
"""

move_bindings = {
    'w': (0.15, 0.0), # (linear_x, angular_z)
    's': (-0.15, 0.0),
    'a': (0.0, 0.15),
    'd': (0.0, -0.15),
    ' ': (0.0, 0.0),
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class VHTeleop(Node):
    def __init__(self):
        super().__init__('vh_teleop_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Teleop Node started. Publishing continuous Twist to /cmd_vel")

    def publish_twist(self, x, z):
        tmsg = Twist()
        tmsg.linear.x = float(x)
        tmsg.angular.z = float(z)
        self.pub.publish(tmsg)
        # self.get_logger().info(f"Published Velocity: Lin={x:.2f}, Ang={z:.2f}")

def main(args=None):
    # Setup terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = VHTeleop()

    print(msg)
    try:
        linear_x = 0.0
        angular_z = 0.0
        while rclpy.ok():
            key = get_key(settings)
            
            if key in move_bindings.keys():
                dx, dz = move_bindings[key]
                # Toggle keys instead of accumulating
                linear_x = dx
                angular_z = dz
                node.publish_twist(linear_x, angular_z)
                print(f"\rCommand: Linear.x={linear_x:.1f}, Angular.z={angular_z:.1f}      ", end="")
            elif key == '\x03': # Ctrl-C
                break
                
            # Keep publishing rate to feed continuous integrations
            if linear_x != 0.0 or angular_z != 0.0:
                 node.publish_twist(linear_x, angular_z)
                 
            # Spin subscriptions non-blockingly
            rclpy.spin_once(node, timeout_sec=0.05)
            
    except Exception as e:
        print(f"\nTeleop error: {e}")
    finally:
        # Send STOP on exit
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
