import ctypes
import numpy as np
import time
import sys
sys.path.append(".")
lib = ctypes.cdll.LoadLibrary('/home/hyperplane/ros-neotic-franka/test_pipeline/build/robotmodule/librobotmodule.so')

def convert_type(input):
    ctypes_map = {
        int: ctypes.c_int64,
        float: ctypes.c_double,
        np.float64: ctypes.c_double,
        np.float32: ctypes.c_double,
        str: ctypes.c_char_p
    }
    input_type = type(input)
    if input_type is list:
        length = len(input)
        if length == 0:
            raise("convert type failed... Input is " + input)
        else:
            arr = (ctypes_map[type(input[0])] * length)()
            for i in range(length):
                arr[i] = bytes(input[i]) if (type(input[0]) is str) else input[i]
            return arr
    elif input_type in ctypes_map:
        return ctypes_map[input_type](bytes(input, encoding="utf-8") if isinstance(input, str) else input)
    else:
        raise("convert type failed... Input is " + input)


class Robot:
    def __init__(self, fci_ip):
        assert isinstance(fci_ip, str)
        address = (ctypes.c_int64 * 1)()
        lib.new_robot(convert_type(fci_ip), address)
        self.obj = int(np.ctypeslib.as_array(address))
        address = (ctypes.c_int64 * 1)()
        lib.new_gripper(convert_type(fci_ip), address)
        self.gripper = int(np.ctypeslib.as_array(address))

    @property
    def RonMoving(self):
        return bool(lib.get_RonMoving())

    @property
    def GonMoving(self):
        return bool(lib.get_GonMoving())

    def stop(self, step=2):
        assert step > 1
        lib.stop_robot(convert_type(int(step)))
        time.sleep(1)

    def start_cartesian_vel_control(self):
        """Start hand Cartesian velocity control"""
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_cartesian_vel_control(convert_type(self.obj))
        
    def start_cartesian_pos_control(self):
        """Start hand Cartesian position control"""
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_cartesian_pos_control(convert_type(self.obj))

    def start_joint_vel_control(self):
        """Start hand joint velocity control"""
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_joint_vel_control(convert_type(self.obj))

    def start_joint_pos_control(self):
        """Start hand joint velocity control"""
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_joint_pos_control(convert_type(self.obj))

    def start_gripper(self):
        """Start moving griper"""
        assert not self.GonMoving, "The Gripper is moving, so you cannot give it another command!"
        lib.start_gripper(convert_type(self.gripper))

    def reset(self):
        assert not self.RonMoving, "The robot is moving, so you cannot reset the robot!"
        lib.reset(convert_type(self.obj))

    def homing(self):
        lib.homing(convert_type(self.gripper))

    def set_gripper_motion(self, width, speed):
        lib.set_gripper_motion(convert_type(float(width)), convert_type(float(speed)))

    def get_gwidth(self):
        return lib.get_gwidth()

    def get_O_T_EE(self):
        state = (ctypes.c_double * 16)()
        lib.get_O_T_EE(state)
        return np.ctypeslib.as_array(state)

    def get_O_dP_EE_d(self):
        O_dP_EE_d = (ctypes.c_double * 6)()
        lib.get_O_dP_EE_d(O_dP_EE_d)
        return np.ctypeslib.as_array(O_dP_EE_d)

    def get_q(self):
        q = (ctypes.c_double * 7)()
        lib.get_q(q)
        return np.ctypeslib.as_array(q)

    def get_dq(self):
        dq = (ctypes.c_double * 7)()
        lib.get_dq(dq)
        return np.ctypeslib.as_array(dq)

    def hand_goto_xyz(self, xyz):
        lib.set_hand_target_xyz(convert_type(list(xyz)))
        print("Finishing seting target xyz")
        lib.start_moving_pos(convert_type(self.obj))

    def update_state(self):
        assert not (self.RonMoving or self.GonMoving), "Cannot update robot's state manually when the robot or the gripper is moving!"
        lib.update_state(convert_type(self.obj), convert_type(self.gripper))

    def set_cartesian_vel(self, vel):
        lib.set_cartesian_vel(convert_type(list(vel)))

    def set_joint_vel(self, vel):
        lib.set_joint_vel(convert_type(list(vel)))

    def __del__(self):
        self.stop()

if __name__ == '__main__':
    r = Robot("172.16.0.2")
    state = r.get_O_T_EE()
    print(state)
    r.reset()
    r.start_joint_vel_control()
    time.sleep(10)
    exit()
    r.start_gripper()
    r.set_gripper_motion(0.02, 0.1)
    t0 = time.time()
    while True:
        width = 0.02 * (np.sin(time.time() - t0) + 1)
        speed = 0.02 * np.abs(np.cos(time.time() - t0))
        r.set_gripper_motion(width, speed)
        time.sleep(0.01)
