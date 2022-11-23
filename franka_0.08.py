import ctypes
import numpy as np
import time
import sys
sys.path.append(".")
lib = ctypes.cdll.LoadLibrary('/home/hyperplane/ros-neotic-franka/test_pipeline/build/robotmodule/librobotmodule.so')
lib.get_gwidth.restype = ctypes.c_double
lib.get_RonMoving.restype = ctypes.c_bool
lib.get_GonMoving.restype = ctypes.c_bool

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
    q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    limit_cushion = 0.01 # q need satisfy q_min + (q_max-q_min) * limit_cushion < q < q_max - (q_max-q_min) * limit_cushion
    def __init__(self, fci_ip, robot_reset_goal: np.ndarray, gripper_reset_goal):
        """
        Create a new Robot instance including a panda robot and a gripper.

        Input:
            - fci_ip: The IP address of the robot;
            - robot_reset_goal: The reset goal of the robot. A numpy array of shape (7,). 
                If you don't know what to set, np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]) is a good choice;
            - gripper_reset_goal: The reset goal of the gripper. A float number.
        """
        assert isinstance(fci_ip, str)
        self.q_max = Robot.q_max - (Robot.q_max - Robot.q_min) * Robot.limit_cushion
        self.q_min = Robot.q_min + (Robot.q_max - Robot.q_min) * Robot.limit_cushion
        assert (self.q_min < robot_reset_goal).all()
        assert (self.q_max > robot_reset_goal).all()
        address = (ctypes.c_int64 * 1)()
        lib.new_robot(convert_type(fci_ip), address, convert_type(list(robot_reset_goal)))
        self.obj = int(np.ctypeslib.as_array(address))
        address = (ctypes.c_int64 * 1)()
        lib.new_gripper(convert_type(fci_ip), address, convert_type(float(gripper_reset_goal)))
        self.gripper = int(np.ctypeslib.as_array(address))

    @property
    def RonMoving(self):
        return bool(lib.get_RonMoving())

    @property
    def GonMoving(self):
        return bool(lib.get_GonMoving())

    @property
    def ExceptionOccurred(self):
        return bool(lib.get_Exception_Occur())

    def stop(self, step=2):
        assert step > 1
        lib.stop_robot(convert_type(int(step)))
        # lib.stop_gripper(convert_type(self.gripper))
        while(self.RonMoving):
            time.sleep(0.1)
        print("Stop funtion finished in Python")

    def stop_gripper(self):
        lib.stop_gripper(convert_type(self.gripper))

    def start_cartesian_vel_control(self):
        """Start hand Cartesian velocity control"""
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_cartesian_vel_control(convert_type(self.obj), convert_type(self.gripper))
        
    def start_cartesian_pos_control(self):
        """Start hand Cartesian position control"""
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_cartesian_pos_control(convert_type(self.obj), convert_type(self.gripper))

    def start_joint_vel_control(self):
        """Start hand joint velocity control"""
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_joint_vel_control(convert_type(self.obj), convert_type(self.gripper))

    def start_joint_pos_control(self):
        """Start hand joint velocity control"""
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        lib.start_joint_pos_control(convert_type(self.obj), convert_type(self.gripper))

    def move_gripper(self, width, speed):
        """
        Start moving griper
        
        Input:
            - width: Intended opening width. [m]
            - speed: Closing speed. [m/s]
        """
        lib.start_gripper(convert_type(self.gripper), convert_type(float(width)), convert_type(float(speed)))

    def reset(self, robot_reset_goal=None, gripper_reset_goal=None):
        assert not self.RonMoving, "The robot is moving, so you cannot reset the robot!"
        if robot_reset_goal is None:
            _robot_reset_goal = 0
        else:
            assert isinstance(robot_reset_goal, np.ndarray) and robot_reset_goal.shape == (7,), "reset_goal should be a numpy array of shape (7,)"
            _robot_reset_goal = convert_type(list(robot_reset_goal.astype(np.float64)))
        if gripper_reset_goal is None:
            gripper_reset_goal = self.get_gwidth()

        lib.reset(convert_type(self.obj), _robot_reset_goal)

    def homing(self):
        lib.homing(convert_type(self.gripper))

    def set_gripper_motion(self, width, speed):
        lib.set_gripper_motion(convert_type(float(width)), convert_type(float(speed)))

    def get_gwidth(self):
        return lib.get_gwidth()

    def get_O_T_EE(self):
        """
        Measured end effector pose in base frame.

        Output: a numpy ndarray of shape (16,) representing a 4x4 matrix in column-major format. 
        """
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

    def hand_goto_xyz(self, xyz, duration=1.0):
        """
        Move hand to a specific position, x_t = (x_d - x_0)sin²(πt/(2duation)) + x_0

        Input:
            - xyz: numpy nparray of shape (3,)
            - duration: time duration of the movement
        """
        assert not self.RonMoving, "The robot is moving, so you cannot give it another command!"
        
        r.start_cartesian_pos_control()
        time.sleep(0.2)
        O_T_EE = self.get_O_T_EE()
        print("start moving from {} to {}".format(O_T_EE[12: 15], xyz))
        start_time = time.time()
        have_print = False
        while (time.time() - start_time < duration):
            if self.ExceptionOccurred:
                # print("PYTHON: exception occurred")
                O_T_EE = self.get_O_T_EE()
                have_print = False
                time.sleep(0.0002)
                start_time = time.time()
                continue
            if not have_print:
                print("start moving from {} to {}".format(O_T_EE[12: 15], xyz))
                have_print = True
            t = time.time() - start_time
            # print("PYTHON: t = ", t)
            x_t = (xyz - O_T_EE[12: 15]) * np.sin(np.pi * t / (2 * duration)) ** 2 + O_T_EE[12: 15]
            current_command = O_T_EE.copy()
            current_command[12: 15] = x_t
            self.set_cartesian_pos(current_command)
            time.sleep(0.0002)
        self.stop(10)

    def update_state(self):
        assert not (self.RonMoving or self.GonMoving), "Cannot update robot's state manually when the robot or the gripper is moving!"
        lib.update_state(convert_type(self.obj), convert_type(self.gripper))

    def set_cartesian_vel(self, vel):
        lib.set_cartesian_vel(convert_type(list(vel)))

    def set_cartesian_pos(self, pos):
        lib.set_cartesian_pos(convert_type(list(pos)))

    def set_joint_vel(self, vel):
        q = self.get_q()
        time = 0.2
        vel = np.clip(vel, (self.q_min - q) / time, (self.q_max - q) / time)
        lib.set_joint_vel(convert_type(list(vel)))

    def __del__(self):
        self.stop()

if __name__ == '__main__':
    r = Robot("172.16.0.2", np.array([-0.559383,0.0185739,0.0232256,-1.84237,-1.07274,3.22823,0.146157]), 0.03)
    # r.reset(np.array([0.292225,-0.108397,-0.698742,-1.99106,-0.921009,3.64708,-0.00322475]))
    r.move_gripper(0.08, 0.1)
    # r.reset(np.array([0.292225,-0.108397,-0.698742,-1.99106,-0.921009,3.64708,-0.00322475]))
    # r.start_joint_vel_control()
    # time.sleep(1)
    # print(r.get_O_T_EE())
    # r.stop_gripper()
    print("Finished!")