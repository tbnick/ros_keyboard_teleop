
import rclpy 
import numpy as np
import openvr as ov

from vr_interface import VRInterface

#standart zur übertragung mittels ros
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped               #brauche ich für die übertragung der velocities
from scipy.spatial.transform import Rotation as R        # eig nur notwendig für die matrixtrafo <- depr?!


# für visualisierung mittels rviz -> tf
import tf2_ros
from geometry_msgs.msg import TransformStamped



class VRInterfaceNode(Node):
    """interface class to access SteamVR's hardware-data and publish it to ros"""
    def __init__(self):
        """Initialisiert die Verbindung zur SteamVR-Runtime"""
        
        super().__init__('vr_interface_node')

        self.interface = VRInterface()


        self.static_broadcaster  = tf2_ros.StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.publish_steamvr_scene_origin()


        self.timer = self.create_timer(
            timer_period_sec= 1.0 / 90.0,
            callback=self.node_loop)

        print("Initialisiere Publisher")
        self.velo_pub = self.create_publisher(
            msg_type=TwistStamped,
            topic='/servo_node/delta_twist_cmd',
            qos_profile=10                                          # rpr depth of msg_history_queue
        )

    def node_loop(self):
        """ROS2 loop - gestartet durch timer.callback"""
        self.interface.update()                                                           # aufruf entspricht exakt der ausgabefrequenz des controllers
        fullpos, mat4x4 = self.interface.get_full_position()
        if fullpos is not None:

            (x,y,z, roll, pitch, yaw) = fullpos
#            linear_velocity, angular_velocity = self.interface._get_velocity()
            grip_pressed = self.interface.is_grip_pressed()
            analog_trigger = self.interface.is_analog_trigger_pressed()

#            self.get_logger().info(
#                f"\n[DATA] Grip: {grip_pressed} | Trigger: {analog_trigger:.2f}\n"
#                f"Pose [XYZ]: {x:.2f}, {y:.2f}, {z:.2f} | [RPY]: {roll:.1f}, {pitch:.1f}, {yaw:.1f}"
#            )
            self.transform_mat_to_msg(mat4x4)

            
#            if linear_velocity is not None:
#                    self.publish_twist(linear_velocity, angular_velocity)
        else:
            self.get_logger().warn('Tracking abgerissen - Controller nicht im Sichtfeld!', throttle_duration_sec=2.0)

    def transform_mat_to_msg(self, mat4x4:np.ndarray)->TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'steamvr_scene_origin'

        # Position aus Matrix extrahieren
        t.transform.translation.x = float(mat4x4[0][3])
        t.transform.translation.y = float(mat4x4[1][3])
        t.transform.translation.z = float(mat4x4[2][3])
            
            # Rotation aus Matrix extrahieren und in Quaternion umwandeln
        rot_matrix = mat4x4[:3, :3]
        rotation = R.from_matrix(rot_matrix)
        q = rotation.as_quat() # Gibt Array im Format [x, y, z, w] zurück
            
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
            
        # Jetzt die Nachricht formgerecht senden
        self.dynamic_broadcaster.sendTransform(t)

        

    def publish_steamvr_scene_origin(self):
            """Verknüpft den impliziten SteamVR-Nullpunkt starr mit der ROS-Welt."""
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'                  # ROS-Simulations-Ursprung
            t.child_frame_id = 'steamvr_scene_origin'    # Der implizite Hardware-Nullpunkt
            
            # Keine Fake-Daten: Identitätsmatrix, da die Hardware-Koordinaten 
            # die absolute Realität abbilden, in die sich RViz hineinrechnet.
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.static_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    try:   
        node = VRInterfaceNode()
        interface = VRInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n [INFO] Testlauf durch Nutzer abgebrochen")
    finally:
        if 'node' in locals():
            interface.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()