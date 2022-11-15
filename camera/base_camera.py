# -*- coding: utf-8 -*-
"""
Created on Sun Nov 13 13:51:05 2022

@author: Deming Chen
@email: cdm@pku.edu.cn
"""
import numpy as np
from typing import Tuple
import transforms3d
from utils import pose_esitmation

class Camera:
    """
    The base class for different types of camera.
    
    We assume that there are three objects in the scene: 
        - this camera;
        - a robot: the frame is called "world frame";
        - a cabinet;

    Besides taking photos, the camera need to detect an aruco marker and return a
    special point related to the marker. All the following attributes need overriding.

    Attributes:
        - intrinsic: the intrinsic parameters of the RGB camera
        - distortion_coefficients: the distortion coefficients of the RGB camera
        - special_point_in_marker: the coordinate of a special point in the marker frame
        - robot_to_cabinet: position of the robot in the cabinet frame
    """
    intrinsic = np.eye(3)
    distortion_coefficients = np.zeros(5)
    special_point_in_marker = np.zeros(3) # the coordinate of a special point in the marker frame
    robot_to_cabinet = np.zeros(3) # xyz_in_cabinet_frame = xyz_in_robot_frame + robot_to_cabinet
    
    def __init__(self, c2w) -> None:
        """
        Initiate a camera.

        Input:
        ------
        c2w: camera pose in the world frame
        """
        self.c2wT = c2w[0]
        quat_xyzw = c2w[1]
        quat_wxyz = np.zeros(4)
        quat_wxyz[0] = quat_xyzw[3]
        quat_wxyz[1:] = quat_xyzw[:3]
        self.c2wR = transforms3d.quaternions.quat2mat(quat_wxyz)

    def reset(self):
        """
        Reset the camera and initial parameters.

        In reinforcement learning, when the environment is reseted, the camera probably
        need reseting. In this scenario, we need to detect a marker and record its position
        to use in the future.

        Input:
        ------
        num_image: the number of RGB images and point clouds need to take during reseting
        """
        for _ in range(50):  # decrease this number will result in the first pointcloud has strange rgb !!!
            rgb, pcd = self.shoot()
        marker_R, marker_T, special_point = self.detect_marker(rgb)
        if marker_R is not None and marker_T is not None and special_point is not None:
            self.marker_R, self.marker_T, self.special_point = marker_R, marker_T, special_point
            self.last_marker_R, self.last_marker_T, self.last_special_point = marker_R, marker_T, special_point
    
    def shoot(self, colorful=False) -> Tuple[np.ndarray, np.ndarray]:
        """
        Take an RGB image and a point cloud

        Input:
        ------
        colorful: if true, the point cloud contains the RGB color of each point (xyzrgb);
        otherwise, it only contains the coordinate of each point.

        Output:
        -------
            - rgb: an numpy ndarray of shape (H, W, 3) and data type \"uint8\"
            - pcd: an numpy ndarray of shape (N, 3) or (N, 6) and data type \"float32\"
        """
        raise NotImplementedError

    def transfer2cabinet(self, xyz_in_camera_frame):
        """
        Input:
        ------
        xyz_in_camera_frame: coordinate of several points in the camera's frame, numpy ndarray of shape (N, 3)

        Output:
        -------
        xyz_in_cabinet_frame: coordinate of several points in the camera's frame, the same shape of the input
        """
        xyz_in_robot_frame = xyz_in_camera_frame @ self.c2wR.T + self.c2wT
        xyz_in_cabinet_frame = xyz_in_robot_frame + self.robot_to_cabinet
        return xyz_in_cabinet_frame

    def detect_marker(self, image):
        """
        Input:
        ------
        image: an RGB image of shape (H, W, 3) and date-type \"uint8\"

        Output:
        -------
            - marker_R: rotation matrix of the marker's frame to the cabinet frame, numpy ndarray of shape (3, 3)
            - marker_T: origin of the marker's frame in the cabinet frame, numpy ndarray of shape (3,)
            - special_point: the coordinate of a special point in the cabinet frame, numpy ndarray of shape (3,)
        """
        marker_in_camera_R, marker_in_camera_t = pose_esitmation(image, self.intrinsic, self.distortion_coefficients) # shape: (3, 3), (1, 3)
        if marker_in_camera_R is None or marker_in_camera_t is None:
            return None, None, None
        special_point_to_camera = marker_in_camera_R @ self.special_point_in_marker[:, None] + marker_in_camera_t.T # shape: (3, 1)
        special_point = self.transfer2cabinet(special_point_to_camera.T)[0]
        marker_origin = self.transfer2cabinet(marker_in_camera_t)[0]
        return self.c2wR @ marker_in_camera_R, marker_origin, special_point


    def get_pointcloud(self, with_rgb=False, with_handle=True):
        """
        Take a pointcloud, and detect the special point

        Output:
            - xyz or xyzrgb: numpy ndarray of shape: (N, 3) or (N, 6)
            - handle_center: numpy ndarray of shape: (3,)
            - marker_R: marker's rotation related to the pose detected when reseting
            - marker_T: marker's displacement related to the position detected when reseting
        """
        rgb, pcd = self.shoot(colorful=with_rgb)
        marker_R, marker_T, handle_center = self.detect_marker(rgb)
        if marker_R is None or marker_T is None or handle_center is None:
            marker_R, marker_T, handle_center = self.last_marker_R, self.last_marker_T, self.last_special_point
        else:
            self.last_marker_R, self.last_marker_T, self.last_special_point = marker_R, marker_T, handle_center
        self.special_point = handle_center

        xyz = self.transfer2cabinet(pcd[:, :3] if with_rgb else pcd)
        mask1 = xyz[:, 2] > 1e-1
        mask2 = xyz[:, 0] < 1
        mask3 = np.abs(xyz[:, 1]) < 0.4
        mask4 = xyz[:, 2] < 0.8
        mask5 = xyz[:, 0] > 0
        mask = mask1 * mask2 * mask3 * mask4 * mask5
        pcd = pcd[mask]

        if with_handle:
            return pcd, handle_center, marker_R @ self.marker_R.T, marker_T - self.marker_T
        else:
            return pcd

    def delete(self):
        """
        Close the camera.

        After finishing an RL task, the camera may need deleting.
        For example, one would like to shut down the hardware after using it.
        """
        raise NotImplementedError

