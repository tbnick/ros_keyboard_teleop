import sys
import select
import termios
import tty

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')               #* initialisierung der node im ros-graphen 
        self.settings = termios.tcgetattr(sys.stdin)

        self.target_joints = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']

        self.latest_live_velocities = [0.0] * 5
        self.vel_step               =  0.4
        self.max_vel                =  1.5

        self.net_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        

        self.joint_limits = {
            'Shoulder_Rotation': [-1.57, 1.57],
            'Shoulder_Pitch':    [-1.57, 1.57],
            'Elbow':             [-1.57, 1.57],
            'Wrist_Pitch':       [-1.57, 1.57],
            'Wrist_Roll':        [-2.00, 2.00]
        }

        self.vel_pub = self.create_publisher(
            msg_type=Float64MultiArray,
            topic='/joint_servo_controller_velocity/commands',                  # hier ist die referenz zu hardware_controllers.yaml
            qos_profile=10
        )  # wir instanziieren nun den publisher, wenn wir einfach über topics kommunizieren wird die latenz radikal verkürzt

        self.joint_state_sub = self.create_subscription(
            msg_type=JointState,
            topic='/joint_states',
            callback=self.joint_state_callback,
            qos_profile=10,
            callback_group=self.net_group
        )

        self.timer = self.create_timer(
            0.02, # 50 Hz Takt-Synchronität zur Hardware (REQ-055)
            self.teleop_loop,
            callback_group=self.timer_group
        )
        self.get_logger().info("=======================================================")
        self.get_logger().info("Echtzeit-Geschwindigkeits-Teleop für SO-101 aktiv.")
        self.get_logger().info("Halte Taste für kontinuierliche Bewegung, lasse los zum Stopp:")
        self.get_logger().info("  Shoulder_Rotation (J0):  [W] / [S]")
        self.get_logger().info("  Shoulder_Pitch    (J1):  [A] / [D]")
        self.get_logger().info("  Elbow             (J2):  [E] / [X]")
        self.get_logger().info("  Wrist_Pitch       (J3):  [R] / [F]")
        self.get_logger().info("=======================================================")
    
    def joint_state_callback(self, msg):
        for i, name in enumerate(self.target_joints):
            if name in msg.name:
                msg_idx = msg.name.index(name)
                self.latest_live_velocities[i] = msg.velocity[msg_idx]
    
    def teleop_loop(self):
        key = self.get_key()
        
        # Der Totmannschalter: Jede Schleife beginnt zwingend im Stillstand!
        cmd_velocities = [0.0] * 5 
        
        if key != '':
            match key:
                case 'w': cmd_velocities[0] = self.vel_step
                case 's': cmd_velocities[0] = -self.vel_step
                case 'a': cmd_velocities[1] = self.vel_step
                case 'd': cmd_velocities[1] = -self.vel_step
                case 'e': cmd_velocities[2] = self.vel_step
                case 'x': cmd_velocities[2] = -self.vel_step
                case 'r': cmd_velocities[3] = self.vel_step
                case 'f': cmd_velocities[3] = -self.vel_step
                case 'q':
                    self.get_logger().warn("Quit-Signal erkannt.")
                    sys.exit(0)

        for i, name in enumerate(self.target_joints):
            min_lim, max_lim = self.joint_limits[name]
            current_pos = self.latest_live_velocities[i]
            
            # Stoppe Achse, wenn sie am oberen Limit klebt und weiter dorthin will
            if current_pos >= (max_lim - 0.05) and cmd_velocities[i] > 0:
                cmd_velocities[i] = 0.0
                self.get_logger().error(f"WARNUNG: {name} blockiert am OBEREN Limit!")
            
            # Stoppe Achse, wenn sie am unteren Limit klebt und weiter dorthin will
            if current_pos <= (min_lim + 0.05) and cmd_velocities[i] < 0:
                cmd_velocities[i] = 0.0
                self.get_logger().error(f"WARNUNG: {name} blockiert am UNTEREN Limit!")

        # REQ-078: Clamping-Sicherheitsschleife
        clamped_velocities = [max(-self.max_vel, min(self.max_vel, v)) for v in cmd_velocities]
        
        self.send_velocity_command(clamped_velocities)

    def send_velocity_command(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.vel_pub.publish(msg)
    
    def get_key(self):
            tty.setraw(sys.stdin.fileno())
            try:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
                if rlist:
                    key = sys.stdin.read(1)
                    while select.select([sys.stdin], [], [], 0.0)[0]:
                        key = sys.stdin.read(1)
                    return key
                return ''
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSANOW, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn('Teleop-Node beendet.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()