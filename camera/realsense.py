import pyrealsense2 as rs
import numpy as np
import cv2
import open3d
import transforms3d
import time
from mani_skill.env.marker import pose_esitmation

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) # Start streaming
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
pipeline.start(config)
pc = rs.pointcloud()
class RealSense():
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
        self.handle_center = None
        self.c2w = c2w
        self.warmup()
        # self.get_pointcloud()

    def warmup(self):
        for i in range(50):  # decrease this number will result in the first pointcloud has strange rgb !!!
            data = self.pipeline.wait_for_frames()
            depth = data.get_depth_frame()
            color = data.get_color_frame()
        colorful = np.asanyarray(color.get_data())
        self.handle_center = self.get_handle_center(colorful)

    def delete(self):
        print("Realsense deleted!!!!!!!!!!!!!")
        self.pipeline.stop()

    def transfer2cabinet(self, xyz_in_camera_frame):
        transition = self.c2w[0]
        quat_xyzw = self.c2w[1]
        quat_wxyz = np.zeros(4)
        quat_wxyz[0] = quat_xyzw[3]
        quat_wxyz[1:] = quat_xyzw[:3]
        R = transforms3d.quaternions.quat2mat(quat_wxyz)
        xyz_world_frame = xyz_in_camera_frame @ R.T + transition 
        xyz_world_frame[:,0] = xyz_world_frame[:,0] - 0.9
        return xyz_world_frame

    def get_handle_center(self, frame):
        # return np.zeros(3)
        handle_center_to_marker = np.array([-0.15, -0.065, 0.01])[:, None]
        marker_to_camera_R, marker_to_camera_t = pose_esitmation(frame)
        #print(marker_to_camera_R.shape,marker_to_camera_t.shape,handle_center_to_marker.shape)
        handle_center_to_camera  = marker_to_camera_R @ handle_center_to_marker + marker_to_camera_t.T
        #print(handle_center_to_camera.shape)
        return self.transfer2cabinet(handle_center_to_camera.T)[0]

    def get_pointcloud(self, with_rgb = False, with_handle=True):
        # time1 = time.time()

        for _ in range(1):
            data = self.pipeline.wait_for_frames()
        depth = data.get_depth_frame()
        color = data.get_color_frame()
        colorful = np.asanyarray(color.get_data())
        handle_center = self.get_handle_center(colorful)
        # handle_mask_ = get_color_mask(self.hsv_low, self.hsv_high, colorful).reshape(-1)
        colorful = colorful.reshape(-1, 3)

        self.pc.map_to(color)
        points = self.pc.calculate(depth)

        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        vtx = self.transfer2cabinet(vtx)
        mask1 = vtx[:, 2] > 1e-1
        mask2 = vtx[:, 0] < 0.15
        mask3 = np.abs(vtx[:, 1]) < 0.4
        mask4 = vtx[:, 2] < 0.8
        mask5 = vtx[:, 0] > -0.7
        mask = mask1 * mask2 * mask3 * mask4 *mask5  

        xyz = vtx[mask]
        rgb = colorful[mask]
        # time2 = time.time()
        # print(f'use {time2 - time1} s for get pointcloud')
        # show_pcd(xyz, rgb, handle_center)
        # print(handle_center)
        if with_rgb:
            return np.concatenate((xyz, rgb), axis=1), handle_center
        else:
            return xyz, handle_center


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


def get_color_mask(hsv_low, hsv_high, image):
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


if __name__ == "__main__":
    print("__name__ == \"__main__\"")
    camera = RealSense(np.array([[-0.24518586365202366, 0.4669115339912172, 0.11584893612944897], [-0.23290171567181367, 0.5206099405741644, -0.7249414414425736, 0.386240840786749]]))
    camera.get_pointcloud()