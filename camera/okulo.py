try:
    import pyokulo as okulo
except ModuleNotFoundError:
    print("You need to install Okulo SDK first ", 
        "and copy \'pyokulo.cpython-3xm-x86_64-linux-gnu.so\' in this directory!")
import cv2 as cv
import numpy as np
import time
from camera import Camera

class Okulo(Camera):
    """Okulo camera for reinforcement learning"""

    def __init__(self, c2w) -> None:
        """
        Initiate a new camera
        
        We need to take an RGB image and a point cloud simultaneously, so we need
        the RGB stream and the ToF stream.
        """
        super().__init__(c2w)
        intrinsic = okulo.intrinsics()
        extrinsic = okulo.extrinsics()
        device = okulo.PDdevice()

        if not device.init():
            print("device init failed")
            exit(-1)
        print("device init succeed")
        self.streams = dict()
        for i in range(1):
            stream = okulo.PDstream(device, i)
            suc = stream.init()
            streamName = stream.getStreamName()
            assert suc, "{} stream init failed!\n".format(streamName)
            print("{} stream init success.".format(streamName))
            assert streamName in ("RGB", "ToF")
            self.streams[streamName] = stream
            # mats = stream.getPyMat()
            # mats = stream.getPyMat()
            start_time = time.time()
            for _ in range(200):
                mats = stream.getPyMat()
            print("Using {} s to take 200 RGB images".format(time.time() - start_time))
            


    def take_rgb(self) -> np.ndarray:
        mats = list()
        while not mats:
            mats = self.streams["RGB"].getPyMat()
            print(123, end='')
        rgb = mats[0]
        return rgb

if __name__ == "__main__":
    camera = Okulo(22)
    rgb = camera.take_rgb()
    print(rgb.shape)