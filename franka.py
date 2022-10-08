import ctypes
import numpy as np
import time

lib = ctypes.cdll.LoadLibrary('build/robotmodule/librobotmodule.so')
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
            raise("convert type failed...input is " + input)
        else:
            arr = (ctypes_map[type(input[0])] * length)()
            for i in range(length):
                arr[i] = bytes(input[i]) if (type(input[0]) is str) else input[i]
            return arr
    elif input_type in ctypes_map:
            # print(type(input), input_type, bytes(input))
            return ctypes_map[input_type](bytes(input, encoding="utf-8") if isinstance(input, str) else input)
    else:
        raise("convert type failed...input is " + input)


class Robot:
    def __init__(self, fci_ip):
        assert isinstance(fci_ip, str)
        self.obj = lib.Robot_new(convert_type(fci_ip))

    def start_moving(self):
        lib.start_moving(self.obj)

    def get_state(self):
        state = (ctypes.c_double * 16)()
        lib.get_O_T_EE_c(state)
        return np.ctypeslib.as_array(state)

    def step(self, action):
        lib.set_action(convert_type(list(action)))

if __name__ == '__main__':
    r = Robot("172.16.0.2")
    state = r.get_state()
    print(type(state), state)
