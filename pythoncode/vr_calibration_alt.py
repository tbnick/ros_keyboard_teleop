import numpy as np
from typing import List

class VRCalibrator:
    """Class for calibration of the table plane for tf
    state machine!
            {0:uncalibrated,
            1:sampling of edge 1,
            2:waiting for sampling of edge 2,
            3:sampling of edge 2,
            4:calibrated}
    """
    def __init__(self):
        self.state:int = 0

        self.points_edge1:List = []
        self.points_edge2:List = []

        self.v1:tuple|None = None           
        self.v2:tuple|None = None
        
        self.origin        = None
        self.T_scene_table:np.ndarray = np.eye(4)

    def _compute_calibration_matrix(self, p0:np.ndarray, p1:np.ndarray, p2:np.ndarray):
        self.origin = p0
        x_axis      = p1 - p0
        x_axis     /= np.linalg.norm(x_axis)
        
        v_y         = p2 - p0
        z_axis      = np.cross(x_axis, v_y)
        z_axis     /= np.linalg.norm(z_axis)

        y_axis      = np.cross(x_axis, z_axis)
        y_axis     /= np.linalg.norm(z_axis)

        self.T_scene_table          = np.eye(4)
        self.T_scene_table[:3, 0]   = x_axis
        self.T_scene_table[:3, 1]   = y_axis
        self.T_scene_table[:3, 2]   = z_axis 
        self.T_scene_table[:3, 3]   = self.origin
        
        return self.T_scene_table

    #! depr
    def process_sample(self, current_pos:tuple, calibration_flag:bool)->None:
        """accepts current_pos and an calibration_flag -> this flag shall be an state, which will be implemented by vr_interface  <- needs to be implemented into vr_cinterface.py
            I want the flag to be an state to be absolutely sure about where in the calib-process we are at any point in time, only having an button_pressed:bool would be ambiguous
            ===> idea got abandoned right after implementation
        """
        match self.state:
            case 0:
                if calibration_flag:
                    self.state = 1
                    self.points_edge1 = [current_pos[:3]]
                    return "[CALIB] Starte Aufnahme von Kante 1...."
            case 1:
                self.points_edge1.append(current_pos[:3])

                if len(self.points_edge1) > 100:
                    self.v1 = self._fit_line3d(np.array(self.points_edge1))
                    self.state = 2
                    return "[CALIB] Kante 1 erfasst, Vektor aufgespannt!"
            case 2:
                if calibration_flag:
                    self.state = 3
                    self.points_edge2 = [current_pos[:3]]
                    return "[CALIB] Starte Aufnahme von Kante 2...."
            case 3:
                self.points_edge2.append(current_pos[:3])

                if len(self.points_edge2) > 100:
                    self.v2 = self._fit_line3d(np.array(self.points_edge2))
                    self._compute_calibration_matrix()
                    self.state = 4
                    return "[CALIB] Kante 2 erfasst, Vektor aufgespannt!"
            case 4:
                return "[CALIB] Kanten erfasst, Vektoren aufgespannt - Basis kann errechnet werden!"
            
    def _fit_line3d(self, points:np.ndarray)->np.ndarray:
        centroid = np.mean(points, axis=0)
        _, _, vh = np.linalg.svd(points-centroid)
        direction = vh[0]
        return direction/np.linalg.norm(direction)


