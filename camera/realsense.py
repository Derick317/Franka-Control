import pyrealsense2 as rs
import numpy as np
import time
from base_camera import Camera

pipeline = rs.pipeline()
align_to = rs.stream.color
alignedFs = rs.align(align_to)
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) # Start streaming
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
pipeline.start(config)
pc = rs.pointcloud()

class RealSense(Camera):
    intrinsic = np.array([[918.2952, 0, 638.9496], [0, 917.5439, 367.0617], [0, 0, 1]])
    distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0]).astype(np.float32)
    special_point_in_marker = np.array([0.085, 0.23, 0.06]) # the coordinate of a special point in the marker frame
    robot_to_cabinet = np.array([0, 0, 0]) # xyz_in_cabinet_frame = xyz_in_camera_frame + robot_to_cabinet
    def __init__(self, c2w):
        self.pipeline = pipeline
        self.pc = pc
        # Configure depth and color streams
        self.name = 'real'
        super().__init__(c2w)
        self.reset()

    def shoot(self, colorful=False):
        data = self.pipeline.wait_for_frames()
        aligned_frames = alignedFs.process(data)
        depth = aligned_frames.get_depth_frame()
        color = aligned_frames.get_color_frame()
        rgb = np.asanyarray(color.get_data())
        if colorful:
            self.pc.map_to(color)
        points = self.pc.calculate(depth)
        xyz = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        pcd = np.concatenate((xyz, rgb), axis=1) if colorful else xyz

        return rgb, pcd

    def delete(self):
        print("Realsense deleted!!!!!!!!!!!!!")
        self.pipeline.stop()


if __name__ == "__main__":
    camera = RealSense(np.array([[-0.17637916136681892, 0.5567354020174904, 0.2488602980557849], [-0.35240212895482415, 0.5218189084973818, -0.6723703853987748, 0.3891475698002122]]))
    xyz, handle, marker_R, marker_T = camera.get_pointcloud()
    print(xyz.shape)
    print(marker_R)