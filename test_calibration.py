import sys

sys.path.append(".")
from franka import Robot
from camera.realsense import RealSense
import open3d # version: 0.15
import numpy as np
# translation: 
#   x: -0.2604876266349236
#   y: 0.4715523716643254
#   z: 0.09823510988541397
# rotation: 
#   x: -0.27049109280501676
#   y: 0.5224533849704154
#   z: -0.6869419351912096
#   w: 0.4266002894066738
robot = Robot("172.16.0.2")
camera = RealSense(np.array([[-0.2604876266349236, 0.4715523716643254, 0.09823510988541397], [-0.27049109280501676, 0.5224533849704154, -0.6869419351912096, 0.4266002894066738]]))

EE = list(robot.get_state()[12: 15])
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