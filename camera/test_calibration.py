import rospy
import numpy as np
import time
import panda_robot
import pybullet as p
import math
from pyquaternion import Quaternion
import transforms3d
rospy.init_node("panda_demo", anonymous=True)
r = panda_robot.PandaArm()
r.enable_robot() 
pos_now, ori_now = r.ee_pose()
#print(pos_now)
r.move_to_neutral()
g = r.get_gripper()
r.get_gripper().open()


# translation: 
#   x: -0.036725630014842026
#   y: 0.37441034353720626
#   z: 0.054031552683774714
# rotation: 
#   x: -0.1953952839457566
#   y: 0.45789330656024263
#   z: -0.8009246387293821
#   w: 0.3326772097623961


c2w = np.array([[-0.036725630014842026, 0.37441034353720626, 0.054031552683774714], [-0.1953952839457566, 0.45789330656024263, -0.8009246387293821, 0.3326772097623961]])
o2c = np.array([[-0.330, 0.192, 1.56], [0, 0, 0, 1]])
o2w = p.multiplyTransforms(c2w[0], c2w[1], o2c[0], o2c[1])


pos = np.asarray(o2w[0])
print(pos)
pos[2] += 0.02
print(pos)
# r.move_to_cartesian_pose(pos, ori_now)
# r.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01])
