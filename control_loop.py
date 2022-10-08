from policy import Policy
from franka import Robot
from plot_traj import Plot, plt
import numpy as np
import time

def main():
    agent = Robot("172.16.0.2")
    policy = Policy(np.array([0.4, 0, 0.2]), 0.04)
    plot = Plot([0, 1], [-0.5, 0.5])
    input("WARNING: This example will move the robot!\n"
          + "Please make sure to have the user stop button at hand!\n"
          + "Press Enter to continue...")
    agent.start_moving()
    print("start moving ... ")
    counter = 0
    while True:
        state = agent.get_state()
        action = policy(state)
        agent.step(action)
        # time.sleep(0.01)
        plt.pause(0.01) # if you want to show the image, choose plt.pause instead of time.sleep
        counter += 1
        if counter % 200 == 0:
            print(f"At {counter} step,\nstate: {state},\naction: {action}")
        plot.update(state[12], state[13])

if __name__ == "__main__":
    main()