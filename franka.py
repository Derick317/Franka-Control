import ctypes
import numpy as np
import time
import sys
sys.path.append(".")
lib = ctypes.cdll.LoadLibrary('/home/hyperplane/ros-neotic-franka/test_pipeline/build/robotmodule/librobotmodule.so')
lib.get_O_T_EE_c.restype = ctypes.POINTER(ctypes.c_double)

def convert_type(input):
    ctypes_map = {
        int: ctypes.c_int,
        float: ctypes.c_double,
        np.float64: ctypes.c_double,
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
        self.obj = lib.Robot_new(convert_type(fci_ip))

    @property
    def OnMoving(self):
        return bool(lib.get_OnMoving())

    def start_cartesian_vel_control(self):
        """Start hand Cartesian velocity control"""
        assert not self.OnMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_cartesian_vel_control(self.obj)
        
    def start_cartesian_pos_control(self):
        """Start hand Cartesian position control"""
        assert not self.OnMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_cartesian_pos_control(self.obj)

    def start_joint_vel_control(self):
        """Start hand joint velocity control"""
        assert not self.OnMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_joint_vel_control(self.obj)

    def start_joint_pos_control(self):
        """Start hand joint velocity control"""
        assert not self.OnMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_joint_pos_control(self.obj)

    def reset(self):
        lib.reset(self.obj)

    def get_state(self):
        state = (ctypes.c_double * 16)()
        lib.get_O_T_EE_c(state)
        return np.ctypeslib.as_array(state)

    def hand_goto_xyz(self, xyz):
        lib.set_hand_target_xyz(convert_type(list(xyz)))
        print("Finishing seting target xyz")
        lib.start_moving_pos(self.obj)

    def update_state(self):
        assert not self.OnMoving, "Cannot update robot's state manually when the robot is moving!"
        lib.update_state(self.obj)

    def step(self, action):
        lib.set_action(convert_type(list(action)))

if __name__ == '__main__':
    r = Robot("172.16.0.2")
    r.update_state()
    print("update")
    print(r.OnMoving)
    state = r.get_state()
    print(state)
    r.reset()
    time.sleep(1)
    r.start_cartesian_pos_control()
    time.sleep(11)
