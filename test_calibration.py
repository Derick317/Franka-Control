import sys

sys.path.append(".")
from franka import Robot
from camera.realsense import RealSense
import open3d # version: 0.15
import numpy as np
# translation: 
#   x: -0.26183748166076815
#   y: 0.49318572450894643
#   z: 0.12896094231876945
# rotation: 
#   x: -0.33163545990407445
#   y: 0.48839349063920445
#   z: -0.6854566235261826
#   w: 0.4261911980549937
robot = Robot("172.16.0.2")
camera = RealSense(np.array([[-0.26183748166076815, 0.49318572450894643, 0.12896094231876945], [-0.33163545990407445, 0.48839349063920445, -0.6854566235261826, 0.4261911980549937]]))

EE = list(robot.get_O_T_EE()[12: 15])
print(EE)
EE[0] -= 0.9
axis = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=EE)
test_pcd = open3d.geometry.PointCloud()
xyz_rgb, handle_center = camera.get_pointcloud(True)
test_pcd.points = open3d.utility.Vector3dVector(xyz_rgb[:, :3])  # 定义点云坐标位置
test_pcd.colors = open3d.utility.Vector3dVector(xyz_rgb[:, 3:] / 255)  # 定义点云的颜色

open3d.visualization.draw_geometries([test_pcd, axis], window_name="Open3D")

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