import time
import rclpy 
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


import sys
import select
import termios
import tty

class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__('arm_teleop_node')               #* initialisierung der node im ros-graphen 

        self.settings = termios.tcgetattr(sys.stdin)
        
        self.target_joints = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']

        self.current_positions = [0.0] * 5
        self.position_step     =  0.01

        self.joint_limits = {
            'Shoulder_Rotation': [-1.57, 1.57],
            'Shoulder_Pitch':    [-1.57, 1.57],
            'Elbow':             [-1.57, 1.57],
            'Wrist_Pitch':       [-1.57, 1.57],
            'Wrist_Roll':        [-2.00, 2.00]
        }

        self._send_goal_future = None
        
        self.net_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        
        self.pos_pub = self.create_publisher(
            msg_type=Float64MultiArray,
            topic='/joint_servo_controller_position/commands',
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
            timer_period_sec=0.02,
            callback=self.teleop_loop,
            callback_group=self.timer_group
        )

        self.get_logger().info("=======================================================")
        self.get_logger().info("MVP Tastatur-Positions-Teleop für SO-100 gestartet.")
        self.get_logger().info("Steuerung per Einzeltastendruck:")
        self.get_logger().info("  Shoulder_Rotation (J0):  [W] positiv  | [S] negativ")
        self.get_logger().info("  Shoulder_Pitch    (J1):  [A] positiv  | [D] negativ")
        self.get_logger().info("  Elbow             (J2):  [E] positiv  | [X] negativ")
        self.get_logger().info("  Wrist_Pitch       (J3):  [R] positiv  | [F] negativ")
        self.get_logger().info("=======================================================")
    
    def joint_state_callback(self, msg):
        if all(p==0.0 for p in self.current_positions):
            for i,name in enumerate(self.target_joints):
                if name in msg.name:
                    msg_idx = msg.name.index(name)
                    self.current_positions[i] = msg.position[msg_idx]

    def teleop_loop(self):
        key = self.get_key()
        if key != '':
            self.process_key_input(key=key)



    def send_position_command(self):
        """#if not self._action_client.wait_for_server(timeout_sec=0.001):
        #    return 
        #
        #goal_msg = FollowJointTrajectory.Goal()
        #goal_msg.trajectory.joint_names = self.target_joints
#
        #point = JointTrajectoryPoint()
        #point.positions = self.current_positions
#
        #point.time_from_start.sec     = 0
        #point.time_from_start.nanosec = int(0.15 * 1e9)
        #
#
        #goal_msg.trajectory.points.append(point)
#
        ## if self._send_goal_future is not None and not self._send_goal_future.done():
        ##    return
        #self._send_goal_future = self._action_client.send_goal_async(goal_msg)"""
        msg = Float64MultiArray()
        msg.data = self.current_positions
        self.pos_pub.publish(msg)


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


    def process_key_input(self, key):
        new_positions = list(self.current_positions)
        match key:
            case 'w':
                new_positions[0] += self.position_step
                self.get_logger().info(f"[Taste W] Inkrementiere Shoulder_Rotation (J0) -> {new_positions[0]}")
            case 's':
                new_positions[0] -= self.position_step
                self.get_logger().info(f"[Taste S] Dekrementiere Shoulder_Rotation (J0) -> {new_positions[0]}")
            case 'a':
                new_positions[1] += self.position_step
                self.get_logger().info(f"[Taste A] Inkrementiere Shoulder_Pitch (J1) -> {new_positions[1]}")
            case 'd':
                new_positions[1] -= self.position_step
                self.get_logger().info(f"[Taste D] Dekrementiere Shoulder_Pitch (J1) -> {new_positions[1]}")
            case 'e':
                new_positions[2] += self.position_step
                self.get_logger().info(f"[Taste E] Inkrementiere Elbow (J2) -> {new_positions[2]}")
            case 'x':
                new_positions[2] -= self.position_step
                self.get_logger().info(f"[Taste X] Dekrementiere Elbow (J2) -> {new_positions[2]}")
            case 'r':
                new_positions[3] += self.position_step
                self.get_logger().info(f"[Taste R] Inkrementiere Wrist_Pitch (J3) -> {new_positions[3]}")
            case 'f':
                new_positions[3] -= self.position_step
                self.get_logger().info(f"[Taste F] Dekrementiere Wrist_Pitch (J3) -> {new_positions[3]}")
            case _:
                # self.get_logger().info(f"Keine Taste gedrückt")
                return # Ignoriere alle anderen Tasten komplett
        
        for i,name in enumerate(self.target_joints):
            min_lim, max_lim = self.joint_limits[name]
            new_positions[i] = max(min_lim, min(max_lim, new_positions[i]))
        
        self.current_positions = new_positions
        self.get_logger().info(f"Soll Winkel (Radians): {self.current_positions}")

        self.send_position_command()




def main(args=None):
    # Initialisierung des globalen rclcpp/rclpy Kontextes
    rclpy.init(args=args)
    
    # Instanziierung unserer Node-Klasse im Speicher
    node = ArmTeleopNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Startet die blockierende Spin-Schleife des Executors
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn('Teleop-Node wird durch Benutzer-Interrupt beendet.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
