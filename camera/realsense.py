import pyrealsense2 as rs
import numpy as np
import cv2
from cv2 import aruco
import open3d
import transforms3d
import time

pipeline = rs.pipeline()
align_to = rs.stream.color
alignedFs = rs.align(align_to)
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) # Start streaming
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
pipeline.start(config)
pc = rs.pointcloud()

class RealSense():
    intrinsic = np.array([[918.2952, 0, 638.9496], [0, 917.5439, 367.0617], [0, 0, 1]])
    distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0]).astype(np.float32)
    special_point_in_marker = np.array([+0.085, 0.23, 0.06]) # the coordinate of a special point in the marker frame
    robot_to_cabinet = np.array([-0, 0, 0]) # xyz_in_cabinet_frame = xyz_in_camera_frame + robot_to_cabinet
    def __init__(self, c2w):
        self.pipeline = pipeline
        # Get device product line for setting a supporting resolution
        # pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        # pipeline_profile = config.resolve(pipeline_wrapper)
        # device = pipeline_profile.get_device()
        # device_product_line = str(device.get_info(rs.camera_info.product_line))
        self.pc = pc
        # Configure depth and color streams
        self.hsv_low = np.array([18, 140, 42])
        self.hsv_high = np.array([85, 199, 92])
        self.name = 'real'
        self.c2w = c2w
        quat_xyzw = self.c2w[1]
        quat_wxyz = np.zeros(4)
        quat_wxyz[0] = quat_xyzw[3]
        quat_wxyz[1:] = quat_xyzw[:3]
        self.c2wR = transforms3d.quaternions.quat2mat(quat_wxyz)
        self.reset()

    def reset(self):
        for _ in range(50):  # decrease this number will result in the first pointcloud has strange rgb !!!
            depth, color = self.get_frames()
        colorful = np.asanyarray(color.get_data())
        # self.handle_center = self.get_handle_center(colorful)
        marker_R, marker_T, special_point = self.detect_marker(colorful)
        if marker_R is not None and marker_T is not None and special_point is not None:
            self.marker_R, self.marker_T, self.special_point = marker_R, marker_T, special_point
            self.last_marker_R, self.last_marker_T, self.last_special_point = marker_R, marker_T, special_point

    def get_frames(self):
        data = self.pipeline.wait_for_frames()
        aligned_frames = alignedFs.process(data)
        depth = aligned_frames.get_depth_frame()
        color = aligned_frames.get_color_frame()
        return depth, color

    def delete(self):
        print("Realsense deleted!!!!!!!!!!!!!")
        self.pipeline.stop()

    def transfer2cabinet(self, xyz_in_camera_frame):
        """
        Input:
            - xyz_in_camera_frame: coordinate of several points in the camera's frame numpy ndarray of shape (N, 3)
        """
        xyz_in_robot_frame = xyz_in_camera_frame @ self.c2wR.T + self.c2w[0]
        xyz_in_cabinet_frame = xyz_in_robot_frame + self.robot_to_cabinet
        # xyz_world_frame[:, 0] = xyz_world_frame[:, 0] - 0.9
        return xyz_in_cabinet_frame

    def get_pointcloud(self, with_rgb=False, with_handle=True):
        """
        Output:
            - xyz or xyzrgb: numpy ndarray of shape: (N, 3) or (N, 6)
            - handle_center: numpy ndarray of shape: (3,)
        """
        # time1 = time.time()
        depth, color = self.get_frames()
        colorful = np.asanyarray(color.get_data())
        marker_R, marker_T, handle_center = self.detect_marker(colorful)
        if marker_R is None or marker_T is None or handle_center is None:
            marker_R, marker_T, handle_center = self.last_marker_R, self.last_marker_T, self.last_special_point
        else:
            self.last_marker_R, self.last_marker_T, self.last_special_point = marker_R, marker_T, handle_center
        self.special_point = handle_center
        colorful = colorful.reshape(-1, 3)

        self.pc.map_to(color)
        points = self.pc.calculate(depth)

        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        vtx = self.transfer2cabinet(vtx)
        mask1 = vtx[:, 2] > 1e-1
        mask2 = vtx[:, 0] < 1
        mask3 = np.abs(vtx[:, 1]) < 0.4
        mask4 = vtx[:, 2] < 0.8
        mask5 = vtx[:, 0] > 0
        mask = mask1 * mask2 * mask3 * mask4 * mask5  

        xyz = vtx[mask]
        rgb = colorful[mask]
        # time2 = time.time()
        # print(f'use {time2 - time1} s for get pointcloud')
        # show_pcd(xyz, rgb, handle_center)
        # print(handle_center)
        result = [np.concatenate((xyz, rgb), axis=1) if with_rgb else xyz]
        if with_handle:
            result += [handle_center, marker_R @ self.marker_R.T, marker_T - self.marker_T]
        return tuple(result)
        
    def detect_marker(self, frame):
        """
        Input: Frame from the video stream

        Output:
            - marker_R: rotation matrix of the marker's frame to the cabinet frame, numpy ndarray of shape (3, 3)
            - marker_T: origin of the marker's frame in the cabinet frame, numpy ndarray of shape (3,)
            - special_point: the coordinate of a special point in the cabinet frame, numpy ndarray of shape (3,)
        """
        marker_in_camera_R, marker_in_camera_t = pose_esitmation(frame, self.intrinsic, self.distortion_coefficients) # shape: (3, 3), (1, 3)
        if marker_in_camera_R is None or marker_in_camera_t is None:
            return None, None, None
        special_point_to_camera = marker_in_camera_R @ self.special_point_in_marker[:, None] + marker_in_camera_t.T # shape: (3, 1)
        special_point = self.transfer2cabinet(special_point_to_camera.T)[0]
        marker_origin = self.transfer2cabinet(marker_in_camera_t)[0]
        return self.c2wR @ marker_in_camera_R, marker_origin, special_point

def show_pcd(xyz, rgb, point=None):
    """
    Plot the point cloud you get.

    Input:
        - xyz: the coordinate of the points in the pointcloud, an numpy ndarray of shape (N, 3)
        - rgb: the color of the points in the pointcloud, an numpy ndarray of shape (N, 3)
        - point: if not None, it is the coordinate of this special point
    """
    test_pcd = open3d.geometry.PointCloud()
    color = np.array([1, 0, 0])[None, :]
    test_pcd.points = open3d.utility.Vector3dVector(xyz)
    test_pcd.colors = open3d.utility.Vector3dVector(rgb / 255)
    if point is not None:
        one_point = open3d.geometry.PointCloud(open3d.utility.Vector3dVector(point[None, :]))
        one_point.colors = open3d.utility.Vector3dVector(color)
    frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    open3d.visualization.draw_geometries([test_pcd, frame] + ([one_point] if point is not None else []))

def hsv_filter(hsv_low, hsv_high, image):
    """
    Input:
        - hsv_low: a numpy array of shape (3,)
        - hsv_high: a numpy array of shape (3,)
        - image: a numpy array of shape (..., 3), and its color ranges from 0 to 255 
    Return:
        mask: a numpy boolean array of shape image.shape[:-1]
    """
    assert (hsv_low <= hsv_high).all()
    assert image.shape[-1] == 3

    dst = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # BGR to HSV
    dst: np.ndarray = cv2.inRange(dst, hsv_low, hsv_high) # dtype: uint8
    assert dst.shape == image.shape[:-1]
    mask = dst.astype(np.bool8)
    return mask

def pose_esitmation(frame, matrix_coefficients, distortion_coefficients):
    '''
    Inputs:
        - frame: frame from the video stream
        - matrix_coefficients: Intrinsic matrix of the calibrated camera
        - distortion_coefficients: Distortion coefficients associated with your camera
    '''
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
    # If markers are detected
    if len(corners) == 1:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.10, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw Axis
            frame= cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec[0], tvec[0], 0.1) 
            R = cv2.Rodrigues(rvec)[0]
            return R , tvec[0]
    else:
        print("Warning: detected {} markers, but 1 marker is needed".format(len(corners)))
        return None, None

if __name__ == "__main__":
    camera = RealSense(np.array([[-0.17637916136681892, 0.5567354020174904, 0.2488602980557849], [-0.35240212895482415, 0.5218189084973818, -0.6723703853987748, 0.3891475698002122]]))
    xyz, handle, marker_R, marker_T = camera.get_pointcloud()
    print(marker_R)