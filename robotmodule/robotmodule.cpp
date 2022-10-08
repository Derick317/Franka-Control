#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <atomic>
#include <iostream>
#include "examples_common.h"
using namespace std;
std::array<double, 6> ACTION = {0, 0, 0, 0, 0, 0};
std::array<double, 6> last_action = {0};
std::array<double, 6> target_action = {0};
constexpr double Epsilon = 1E-4;
constexpr double Duration = 4E-2;
double oteec[16] = {0};
double time_ = 0;

franka::CartesianVelocities velocity_callback(const franka::RobotState &robot_state, franka::Duration period)
{
    // std::cout << "callback" << std::endl;
    time_ += period.toSec();
    for (int i = 0; i < 16; i++)
    {
        oteec[i] = robot_state.O_T_EE_c[i];
    }
    if (time_ > Duration)
    {
        time_ = 0;
        for (int i = 0; i < 6; i++){
            last_action[i] = target_action[i];
            target_action[i] = ACTION[i];
        }
    }
    double v_x = last_action[0] + (target_action[0] - last_action[0]) * time_ / Duration;
    double v_y = last_action[1] + (target_action[1] - last_action[1]) * time_ / Duration;
    double v_z = last_action[2] + (target_action[2] - last_action[2]) * time_ / Duration;

    franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};
    return output;
}

void *robot_run(void *args)
{
    franka::Robot *robot = (franka::Robot *)(args);
    robot->control(velocity_callback);
}

extern "C"
{
    franka::Robot *Robot_new(char fci_ip[])
    {
        franka::Robot *robot = new franka::Robot(fci_ip);
        setDefaultBehavior(*robot);
        return robot;
    }

    void start_moving(franka::Robot *robot)
    {
        pthread_t tids[1];
        pthread_create(&tids[0], NULL, robot_run, robot);
    }

    void get_O_T_EE_c(double* state)
    {
        for (int i = 0; i < 16; i++)
            state[i] = oteec[i];
        return;
    }

    void set_action(double *action)
    {
        for (int i = 0; i < 6; i++)
        {
            ACTION[i] = action[i];
        }
    }
}