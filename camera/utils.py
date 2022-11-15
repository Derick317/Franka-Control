import numpy as np
import open3d
import cv2
from cv2 import aruco

def depth2pc(depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """
    Convert depth image to point cloud

    NOTE: the depth means the distance between the camera and a point, not its projection on z!

    Input:
    ------
        - depth: a depth image, numpy ndarray of shape (H, W);
        - K: intrinsic matrix of the camera, numpy ndarray of shape (3, 3).
    
    Output:
    -------
    A point cloud, numpy ndarray of shape (H * W, 3). It is represented as coordinates of all
    points in the camera's frame.
    """
    H, W = depth.shape
    row = np.array(range(H)).reshape(H, 1, 1)
    col = np.array(range(W)).reshape(1, W, 1)
    rows = row.repeat(W, axis=1)
    cols = col.repeat(H, axis=0)
    pixel = np.concatenate((cols, rows, np.ones((H, W, 1))), axis=2)
    xyz = (np.linalg.inv(K) @ pixel.reshape(-1, 3).T) # shape: (3, H*W)
    dpt = depth.flatten()
    
    pc = (xyz * dpt / np.linalg.norm(xyz, axis=0)).T
    return pc

def pose_esitmation(frame, matrix_coefficients, distortion_coefficients):
    '''
    Detect a marker in an image

    Inputs:
    -------
        - frame: an RGB image of shape (H, W, 3) and date-type \"uint8\"
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
            # Estimate pose of each marker and return the values rvec and tvec
            # different from those of camera coefficients
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.10, 
                matrix_coefficients, distortion_coefficients)
            # Draw Axis
            frame= cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec[0], tvec[0], 0.1) 
            R = cv2.Rodrigues(rvec)[0]
            return R , tvec[0]
    else:
        print("Warning: detected {} markers, but 1 marker is needed".format(len(corners)))
        return None, None

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
    ------
        - hsv_low: a numpy array of shape (3,)
        - hsv_high: a numpy array of shape (3,)
        - image: a numpy array of shape (..., 3), and its color ranges from 0 to 255

    Return:
    -------
        mask: a numpy boolean array of shape image.shape[:-1]
    """
    assert (hsv_low <= hsv_high).all()
    assert image.shape[-1] == 3

    dst = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # BGR to HSV
    dst: np.ndarray = cv2.inRange(dst, hsv_low, hsv_high) # dtype: uint8
    assert dst.shape == image.shape[:-1]
    mask = dst.astype(np.bool8)
    return mask