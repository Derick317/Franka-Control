#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <iostream>
#include "examples_common.h"
#include "callback_function.h"
using namespace std;

bool OnMoving = false; // Whether a control or motion generator loop is active

struct cartesian_vel_control_data{
   franka::Robot *robot;
   std::function<franka::CartesianVelocities(const franka::RobotState &, franka::Duration)> callback;
};

struct cartesian_pos_control_data{
   franka::Robot *robot;
   std::function<franka::CartesianPose(const franka::RobotState &, franka::Duration)> callback;
};

struct joint_vel_control_data{
   franka::Robot *robot;
   std::function<franka::JointVelocities(const franka::RobotState &, franka::Duration)> callback;
};

struct joint_pos_control_data{
   franka::Robot *robot;
   std::function<franka::JointPositions(const franka::RobotState &, franka::Duration)> callback;
};

void *run_cartesian_vel(void *args)
{
    cartesian_vel_control_data *data = (cartesian_vel_control_data *)args;
    franka::Robot *robot = data->robot;
    OnMoving = true;
    robot->control(data->callback);
    OnMoving = false;
    delete data;
}

void *run_cartesian_pos(void *args)
{
    cartesian_pos_control_data *data = (cartesian_pos_control_data *)args;
    franka::Robot *robot = data->robot;
    OnMoving = true;
    robot->control(data->callback);
    OnMoving = false;
    cout << "Just after running" << endl;
}

void *run_joint_vel(void *args)
{
    joint_vel_control_data *data = (joint_vel_control_data *)args;
    franka::Robot *robot = data->robot;
    OnMoving = true;
    robot->control(data->callback);
    OnMoving = false;
    delete data;
}


void *run_joint_pos(void *args)
{
    joint_pos_control_data *data = (joint_pos_control_data *)args;
    franka::Robot *robot = data->robot;
    OnMoving = true;
    robot->control(data->callback);
    OnMoving = false;
    delete data;
}

extern "C"
{
    franka::Robot *Robot_new(char fci_ip[]);
    
    void start_cartesian_pos_control(franka::Robot *robot);

    void start_cartesian_vel_control(franka::Robot *robot);

    void start_joint_pos_control(franka::Robot *robot);

    void start_joint_vel_control(franka::Robot *robot);

    void reset(franka::Robot *robot);

    void set_hand_target_xyz(double* xyz);

    void update_state(franka::Robot *robot);
    
    void get_O_T_EE_c(double* state);

    void set_action(double *action);

    bool get_OnMoving();
}

franka::Robot *Robot_new(char fci_ip[])
{
    franka::Robot *robot = new franka::Robot(fci_ip);
    CurrentState = robot->readOnce();
    
    return robot;
}

void start_cartesian_vel_control(franka::Robot *robot)
{
    setDefaultBehavior(*robot);
    pthread_t tids[1];
    cartesian_vel_control_data *args = new cartesian_vel_control_data;
    args->callback = velocity_callback;
    args->robot = robot;
    pthread_create(&tids[0], NULL, run_cartesian_vel, args);
}

void start_cartesian_pos_control(franka::Robot *robot)
{
    setDefaultBehavior(*robot);
    pthread_t tids[1];
    cartesian_pos_control_data *args = new cartesian_pos_control_data;
    args->callback = test_cartesian_pos;
    args->robot = robot;
    pthread_create(&tids[0], NULL, run_cartesian_pos, args);
    cout << "Out thread" << endl;
}

void start_joint_pos_control(franka::Robot *robot)
{
    setDefaultBehavior(*robot);
    pthread_t tids[1];
    joint_pos_control_data *args = new joint_pos_control_data;
    args->callback = joint_pos_callback;
    args->robot = robot;
    pthread_create(&tids[0], NULL, run_joint_pos, args);
}

void start_joint_vel_control(franka::Robot *robot)
{
    setDefaultBehavior(*robot);
    pthread_t tids[1];
    joint_vel_control_data *args = new joint_vel_control_data;
    args->callback = joint_vel_callback;
    args->robot = robot;
    pthread_create(&tids[0], NULL, run_joint_vel, args);
}

void reset(franka::Robot *robot)
{
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    OnMoving = true;
    robot->control(motion_generator);
    OnMoving = false;
}

void update_state(franka::Robot *robot)
{
    CurrentState = robot->readOnce();
}

void get_O_T_EE_c(double* state)
{
    for (int i = 0; i < 16; i++)
        state[i] = CurrentState.O_T_EE_c[i];
    return;
}

void set_action(double *action)
{
    for (int i = 0; i < 6; i++)
    {
        ACTION[i] = action[i];
    }
}

bool get_OnMoving()
{
    return OnMoving;
}