
import rclpy 
import numpy as np
import openvr as ov
from typing import List

from vr_interface import VRInterface
from vr_calibration import VRCalibrator

#standart zur übertragung mittels ros
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped               #brauche ich für die übertragung der velocities
from scipy.spatial.transform import Rotation as R        # eig nur notwendig für die matrixtrafo <- depr?!

# für visualisierung mittels rviz -> tf
import tf2_ros
from geometry_msgs.msg import TransformStamped



class InterfaceCommunicator(Node):
    """interface class to access SteamVR's hardware-data and publish it to ros"""
    def __init__(self):
        """Initialisiert die Verbindung zur SteamVR-Runtime"""
        
        super().__init__('InterfaceCommunicator')

        self.interface  = VRInterface()
        self.calibrator = VRCalibrator()


        self.static_broadcaster  = tf2_ros.StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.publish_steamvr_scene_origin()

        self.T_scene_table = self.execute_calibration()
        self.publish_calibrated_table(self.T_scene_table)


        self.timer = self.create_timer(
            timer_period_sec= 1.0 / 90.0,
            callback=self.node_loop)

        self.get_logger().info("Initialisiere Publisher")
        self.velo_pub = self.create_publisher(
            msg_type=TwistStamped,
            topic='/servo_node/delta_twist_cmd',
            qos_profile=10                                          # rpr depth of msg_history_queue
        )
    
    def execute_calibration(self)->np.ndarray:
        points:List = []
        prompts = [
                "Platziere den Controller auf dem gewünschten TISCH-URSPRUNG (P0) und drücke ENTER...",
                "Platziere den Controller auf der RECHTEN TISCHKANTE (P1, Richtung X-Achse) und drücke ENTER...",
                "Platziere den Controller auf der HINTEREN TISCHKANTE (P2, Richtung Y-Achse) und drücke ENTER..."
            ]
        
        for prompt in prompts:
            self.get_logger().info(prompt)
            while rclpy.ok():

                self.interface.update()
                tracking  = self.interface.get_full_position()

                if (tracking is not None and self.interface.is_grip_pressed()):
                    _, mat4x4 = tracking
                    t_ros, _  = self.coordinate_transform(mat4x4)
                    points.append(t_ros)
                    self.get_logger().info(f"-> Punkt {t_ros} erfolgreich gespeichert.")
                    while self.interface.is_grip_pressed() and rclpy.ok():
                        self.interface.update()
                        self.get_logger().info(f'warte auf bestätigung - trackpad_touched {self.interface.is_trackpad_touched()}')
                    break
                if tracking is None:
                    self.get_logger().warn('Tracking abgerissen!', throttle_duration_sec=2.0)
                        # self.shutdown()
                

        return self.calibrator._compute_calibration_matrix(points[0], points[1], points[2])
    
    def node_loop(self):
        """ROS2 loop - gestartet durch timer.callback"""
        self.interface.update()                                                           # aufruf entspricht exakt der ausgabefrequenz des controllers
        tracking_result = self.interface.get_full_position()
        
        if tracking_result is not None:
            _, mat4x4 = tracking_result

            tr_mat, Rros_mat = self.coordinate_transform(mat4x4)
            
            T_scene_controller = np.eye(4)
            T_scene_controller[:3, :3] = Rros_mat
            T_scene_controller[:3, 3] = tr_mat

            T_table_scene = np.linalg.inv(self.T_scene_table)
            T_table_controller = T_table_scene @ T_scene_controller
            
            self.transform_mat_to_msg(mat4x4=T_table_controller, child_frame='vr_controller')
        else:
            self.get_logger().warn('Tracking abgerissen - Controller nicht im Sichtfeld!', throttle_duration_sec=2.0)

        lighthouses = self.interface.get_lighthouse_poses()
        for lh_name, lh_matrix in lighthouses.items():
            self.transform_mat_to_msg(lh_matrix, child_frame=lh_name)


    def transform_mat_to_msg(self, mat4x4:np.ndarray, child_frame:str)->TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'calibrated_table_origin'                  # ROS-Simulations-Ursprung
        t.child_frame_id = child_frame                              # Der implizite Hardware-Nullpunkt


        # Position aus Matrix extrahieren
        t.transform.translation.x = float(mat4x4[0,3])
        t.transform.translation.y = float(mat4x4[1,3])
        t.transform.translation.z = float(mat4x4[2,3])
            
        # Rotation aus Matrix extrahieren und in Quaternion umwandeln
        rotation = R.from_matrix(mat4x4[:3, :3])
        q = rotation.as_quat() # Gibt Array im Format [x, y, z, w] zurück
            
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
            
        # Jetzt die Nachricht formgerecht senden
        self.dynamic_broadcaster.sendTransform(t)

    def coordinate_transform(self, measurement_mat:np.ndarray):
        # Konstante Basiswechselmatrix (OpenVR Y-Up -> ROS Z-Up)
        R_OVR_ROS = np.array([
            [ 0.0, 0.0, -1.0],  # ROS X ist OpenVR -Z
            [-1.0, 0.0,  0.0],  # ROS Y ist OpenVR -X
            [ 0.0, 1.0,  0.0]   # ROS Z ist OpenVR  Y
        ])

        R_ovr = measurement_mat[:3, :3]
        t_ovr = measurement_mat[:3, 3]

        # Mathematischer Basiswechsel (Tensortransformation)
        t_ros = R_OVR_ROS @ t_ovr
        R_ros = R_OVR_ROS @ R_ovr @ R_OVR_ROS.T

        return (t_ros, R_ros)

    def publish_steamvr_scene_origin(self):
        """Verknüpft den impliziten SteamVR-Nullpunkt starr mit der ROS-Welt."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'steamvr_scene_origin'
        
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

    def publish_calibrated_table(self, T_matrix:np.ndarray):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "steamvr_scene_origin"
        t.child_frame_id  = "calibrated_table_origin"

        t.transform.translation.x = float(T_matrix[0, 3])
        t.transform.translation.y = float(T_matrix[1, 3])
        t.transform.translation.z = float(T_matrix[2, 3])

        rotation = R.from_matrix(T_matrix[:3, :3])
        q        = rotation.as_quat()                                         # x,y,z,w
    
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.static_broadcaster.sendTransform(t)
        

    def shutdown(self):
        self.get_logger().info("[VRInterfaceNode] Shutting down...")
        self.interface.shutdown()

def main():
    rclpy.init()
    try:   
        node = InterfaceCommunicator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n [INFO] Testlauf durch Nutzer abgebrochen")
    finally:
        if node is not None:
            node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()