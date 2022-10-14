from policy import Policy
from franka import Robot
from plot_traj import Plot, plt
import numpy as np
import time

def main():
    agent = Robot("172.16.0.2")
    policy = Policy(np.array([0.4, 0, 0.2]), 0.04)
    # plot = Plot([0, 1], [-0.5, 0.5])
    input("WARNING: This example will move the robot!\n"
          + "Please make sure to have the user stop button at hand!\n"
          + "Press Enter to continue...")
    agent.reset()
    agent.start_cartesian_vel_control()
    print("start moving ...\nIf you plot an figure, close the figure first then press Ctrl+C to stop the robot")
    # agent.homing()
    # agent.start_gripper()
    # agent.set_gripper_motion(0.02, 0.1)
    time.sleep(0.5)
    # t0 = time.time()
    for _ in range(100):
        state = agent.get_O_T_EE()
        action = policy(state)
        agent.set_cartesian_vel(action)
        # width = 0.02 * (np.sin(time.time() - t0) + 1)
        # speed = 0.02 * np.abs(np.cos(time.time() - t0))
        # agent.set_gripper_motion(width, speed)
        time.sleep(0.01)
        # plt.pause(0.01) # if you want to show the image, choose plt.pause instead of time.sleep
        # plot.update(state[12], state[13])
    agent.stop(10)
    print("Finished once")
    agent.update_state()
    # agent.reset()
    agent.start_cartesian_vel_control()
    print("Another traj")
    for _ in range(100):
        # print(f"Step {_} in the 2nd loop")
        state = agent.get_O_T_EE()
        action = policy(state)
        agent.set_cartesian_vel(action)
        time.sleep(0.01)
        # plt.pause(0.01) # if you want to show the image, choose plt.pause instead of time.sleep
        # plot.update(state[12], state[13])
    agent.stop(10)

    
if __name__ == "__main__":
    main()