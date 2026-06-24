
import rclpy 
import numpy as np
import openvr as ov

#standart zur übertragung mittels ros
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped               #brauche ich für die übertragung der velocities
from scipy.spatial.transform import Rotation as R        # eig nur notwendig für die matrixtrafo <- depr?!


# für visualisierung mittels rviz -> tf
import tf2_ros
from geometry_msgs.msg import TransformStamped



class VRInterface():
    """interface class to access SteamVR's hardware-data"""
    def __init__(self):
        """Initialisiert die Verbindung zur SteamVR-Runtime"""
        # hier kümmern wir uns um die OVR-interna
        print("Verbinde mit Steam-VR...")
        try:
            self.vr = ov.init(ov.VRApplication_Scene)
            self.controller_index = None
            self.poses = None
            print("SteamVR erfolgreich verbunden!")
        except ov.OpenVRError as e:
            raise RuntimeError(f"Hardware-Fehler: SteamVR nicht erreicht ({e})")

    # ab hier ist alles ovr - auslesen, updaten etc
    def update(self):
        """Holt den neusten Dataframe. Muss 1x pro Main-Loop ausgeführt werden"""
        if not self.vr:
            return 
        
        self.poses = self.vr.getDeviceToAbsoluteTrackingPose(
                ov.TrackingUniverseStanding, 0, ov.k_unMaxTrackedDeviceCount)

        if not self.controller_index:
            for i in range(ov.k_unMaxTrackedDeviceCount):
                device_class = self.vr.getTrackedDeviceClass(i)
                if device_class == ov.TrackedDeviceClass_Controller:
                    self.controller_index = i
                    print(f"[VRInterface] Controller mit Index {i} gefunden.")
                    break

    def shutdown(self):
        """Trennt die Verbindung sicher"""
        print("[VRInterface] Beende Verbindung")
        ov.shutdown()
        self.vr = None

    def get_full_position(self):
        """Liest aktuellen Zustand des Controllers aus (X;Y;Z;Roll;Pitch;Yaw)"""
        if not self.controller_index or not self.poses:
            return None

        pose = self.poses[self.controller_index]
        if pose.bPoseIsValid:
            m       = pose.mDeviceToAbsoluteTracking
            mat4x4  = self._pad_position_matrix(matrix=m)
            angles  = self._get_euler_angles(mat4x4=mat4x4)
            return (mat4x4[0][3], mat4x4[1][3], mat4x4[2][3], angles[0], angles[1], angles[2]), mat4x4
        else: return None

    def _get_velocity(self):
        pose = self.poses[self.controller_index]
        if not pose.bPoseIsValid: return None, None

        v, phi = pose.vVelocity, pose.vAngularVelocity
        linear_velocity  = np.array([v[0], v[1], v[2]])
        angular_velocity = np.array([phi[0], phi[1], phi[2]])

        return linear_velocity, angular_velocity

    def get_trackpad_position(self):
        """Prüft ob Trackpad betätigt wird.
            returns [x,y]
            depr. kwargs: threshold-> was used to decipher wether the button was excited based on an value reference 
            --> now we only return the analog excitation of the analog-trigger
        """
        if not self.controller_index:
            return (0.0, 0.0)
        
        result, state = self.vr.getControllerState(self.controller_index)
        if result:
            return (float(state.rAxis[0].x), float(state.rAxis[0].y))
        else: return (0.0, 0.0)

    def is_analog_trigger_pressed(self):
        """Prüft ob Analogen-Trigger ausschlägt.
            depr. kwargs: threshold-> was used to decipher wether the button was excited based on an value reference 
            --> now we only return the analog excitation of the analog-trigger
        """
        if not self.controller_index:
            return 0.0
        
        result, state = self.vr.getControllerState(self.controller_index)
        if result:
            return float(state.rAxis[1].x)
        else: return 0.0

    def is_trackpad_pressed(self)->bool:
        """checks for excitation of grip-buttons. Uses Bitwise-Op to extract state from 64-bit-integer"""
        if not self.controller_index:
            return False
        result, state = self.vr.getControllerState(self.controller_index)
        if result:
            gripmask = (1<<ov.k_EButton_SteamVR_Touchpad)
            return (state.ulButtonPressed & gripmask)!=0
        return False
        
    def is_grip_pressed(self)->bool:
        """checks for excitation of grip-buttons. Uses Bitwise-Op to extract state from 64-bit-integer"""
        if not self.controller_index:
            return False
        result, state = self.vr.getControllerState(self.controller_index)
        if result:
            gripmask = (1<<ov.k_EButton_Grip)
            return (state.ulButtonPressed & gripmask)!=0
        return False
    
    def is_trackpad_touched(self)->bool:
        """checks for excitation of grip-buttons. Uses Bitwise-Op to extract state from 64-bit-integer"""
        if not self.controller_index:
            return False
        result, state = self.vr.getControllerState(self.controller_index)
        if result:
            gripmask = (1<<ov.k_EButton_SteamVR_Touchpad)
            return (state.ulButtonTouched & gripmask)!=0
        return False
    
    def _pad_position_matrix(self, matrix: np.ndarray)->np.ndarray:
        """takes an given 3x4 matrix from controller-position and pads it to the standardized shape of transformation-matrix"""
        mat4x4 = np.array([
            [matrix[0][0],matrix[0][1], matrix[0][2], matrix[0][3]],
            [matrix[1][0],matrix[1][1], matrix[1][2], matrix[1][3]],
            [matrix[2][0],matrix[2][1], matrix[2][2], matrix[2][3]],
            [0.0,  0.0,    0.0,  1.0]])
        return mat4x4

    def _get_euler_angles(self, mat4x4:np.ndarray):
        """extracts angles from 4x4-matrix using scipy's rotation package"""
        rot_matrix = mat4x4[:3, :3]
        rotation   = R.from_matrix(rot_matrix)
        # VR-Standard-Reihenfolge: Y (Yaw) -> X (Pitch) -> Z (Roll) <--- rechtshändiges Koordinaten
        yaw, pitch, roll = rotation.as_euler('yxz', degrees=True)
        return roll, pitch, yaw

    def get_lighthouse_poses(self):
        """Sucht nach allen aktiven Lighthouses und gibt deren 4x4-Matrizen zurück."""
        if not self.poses:
            return {}
        
        lighthouse_data = {}
        for i in range(ov.k_unMaxTrackedDeviceCount):
            device_class = self.vr.getTrackedDeviceClass(i)
            # Prüfen, ob das Gerät eine Tracking-Referenz (Lighthouse) ist
            if device_class == ov.TrackedDeviceClass_TrackingReference:
                pose = self.poses[i]
                if pose.bPoseIsValid:
                    m = pose.mDeviceToAbsoluteTracking
                    mat4x4 = self._pad_position_matrix(matrix=m)
                    lighthouse_data[f"lighthouse_{i}"] = mat4x4
        return lighthouse_data

if __name__ == "__main__":
    pass