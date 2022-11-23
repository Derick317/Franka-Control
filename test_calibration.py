import sys

sys.path.append(".")
from franka import Robot
from camera.realsense import RealSense
import open3d # version: 0.15
import numpy as np
#! it is a reference of the camera coordinate system
# translation: 
#   x: -0.07384213513610163
#   y: 0.507094439338527
#   z: 0.3727735948655472
# rotation: 
#   x: -0.367724224822546
#   y: 0.5153029023474898
#   z: -0.6295658625984545
#   w: 0.4504316129686896
robot = Robot("172.16.0.2", np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]), 0.03)
camera = RealSense(np.array([[-0.07384213513610163, 0.507094439338527, 0.3727735948655472], [-0.367724224822546, 0.5153029023474898, -0.6295658625984545, 0.4504316129686896]]))

EE = robot.get_O_T_EE()[12: 15]
print("End Effector: ", EE + RealSense.robot_to_cabinet)
EE += RealSense.robot_to_cabinet
axis = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=list(EE))
test_pcd = open3d.geometry.PointCloud()
xyz_rgb, handle_center, marker_R, marker_T = camera.get_pointcloud(True)
test_pcd.points = open3d.utility.Vector3dVector(xyz_rgb[:, :3])  # 定义点云坐标位置
test_pcd.colors = open3d.utility.Vector3dVector(xyz_rgb[:, 3:] / 255)  # 定义点云的颜色
print("Handle Center: ", handle_center)
open3d.visualization.draw_geometries([test_pcd, axis], window_name="Open3D")

np.savetxt("read_pcd.txt", xyz_rgb[:, :3])

# vis = open3d.visualization.Visualizer()
# vis.create_window(window_name="Open3D")
# vis.get_render_option().point_size = 5
# vis.add_geometry(axis)
# vis.add_geometry(test_pcd)

# while True:
#     xyz_rgb, handle_center = camera.get_pointcloud(True)
#     test_pcd.points = open3d.utility.Vector3dVector(xyz_rgb[:, :3])  # 定义点云坐标位置
#     test_pcd.colors = open3d.utility.Vector3dVector(xyz_rgb[:, 3:] / 255)  # 定义点云的颜色
#     vis.update_geometry(test_pcd)
#     vis.update_geometry(axis)
#     vis.poll_events()
#     vis.update_renderer()