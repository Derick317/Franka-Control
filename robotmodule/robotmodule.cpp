#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <unistd.h>
#include <iostream>
#include "examples_common.h"
#include "callback_function.h"
#include "robotmodule.h"
using namespace std;


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

void handle_exception(franka::Robot *robot, const franka::Exception &e)
{
    ExceptionOccur = 1;
    std::cout << e.what() << "\nTrying recovery..." << std::endl;
    try
    {
        robot->automaticErrorRecovery();
    }
    catch (const franka::Exception &e)
    {
        std::cout << e.what() << std::endl;
        std::cout << "Error recovery unsuccessful" << std::endl;
    }
    sleep(0.2);
    reset(robot);
    CurrentRobotState = robot->readOnce();
    sleep(0.2);
    ExceptionOccur = 0;
}

void *run_cartesian_vel(void *args)
{
    cartesian_vel_control_data *data = (cartesian_vel_control_data *)args;
    franka::Robot *robot = data->robot;
    try
    {
        CartesianVeloticy = {0};
        RonMoving = true;
        std::cout << "Just before Cartesian velocity control" << std::endl;
        robot->control(data->callback);
    }
    catch(const franka::ControlException& e)
    {
        handle_exception(robot, e);
    }
    catch(const franka::NetworkException& e)
    {
        handle_exception(robot, e);
    }
    RonMoving = false;
    std::cout << "Just after Cartesian velocity control" << std::endl;
    return nullptr;
}

void *run_cartesian_pos(void *args)
{
    cartesian_pos_control_data *data = (cartesian_pos_control_data *)args;
    franka::Robot *robot = data->robot;
    CurrentRobotState = robot->readOnce();
    RonMoving = true;
    bool success = false;
    while(!success)
        try
        {
            CartesianPosition = last_cartesian_pos = target_cartesian_pos = CurrentRobotState.O_T_EE_c;
            for(int i=12; i<15; i++)
                std::cout << CartesianPosition[i] << ' ';
            time_ = 0;
            std::cout << "Just before Cartesian position control" << std::endl;
            robot->control(data->callback);
            cout << "Just after running" << endl;
            success = true;
        }
        catch(const franka::ControlException& e)
        {
            handle_exception(robot, e);
        }
        catch(const franka::NetworkException& e)
        {
            handle_exception(robot, e);
        }
    RonMoving = false;
    
    return NULL;
}

void *run_joint_vel(void *args)
{
    joint_vel_control_data *data = (joint_vel_control_data *)args;
    franka::Robot *robot = data->robot;
    try
    {
        JointVelocity = {0};
        RonMoving = true;
        std::cout << "Just before joint velocity control" << std::endl;
        robot->control(data->callback);
    }
    catch(const franka::ControlException& e)
    {
        handle_exception(robot, e);
    }
    catch(const franka::NetworkException& e)
    {
        handle_exception(robot, e);
    }
    std::cout << "Just after joint velocity control" << std::endl;
    RonMoving = false;
    return nullptr;
}

void *run_joint_pos(void *args)
{
    joint_pos_control_data *data = (joint_pos_control_data *)args;
    franka::Robot *robot = data->robot;
    RonMoving = true;
    std::cout << "Just before joint position control" << std::endl;
    robot->control(data->callback);
    std::cout << "Just after joint position control" << std::endl;
    RonMoving = false;
    return NULL;
}

void *move_gripper(void *args)
{
    franka::Gripper *gripper = (franka::Gripper *)args;
    GonMoving = true;
    while(true)
    {
        gripper->move(GripperTarget, GripperSpeed);
        CurrentGripperState = gripper->readOnce();
    }
    GonMoving = false;
}

void new_robot(char fci_ip[], void **address)
{
    franka::Robot *robot = new franka::Robot(fci_ip);
    STOP = 0;
    CurrentRobotState = robot->readOnce();
    cout << "Robot address: "<< robot << endl;
    *address = robot;
}

void new_gripper(char fci_ip[], void **address)
{
    franka::Gripper *gripper = new franka::Gripper(fci_ip);
    CurrentGripperState = gripper->readOnce();
    *address = gripper;
}

void stop_robot(long step)
{
    // cout << "tids[0] = " << tids[0] << endl;
    if (RonMoving)
    {   
        cout << "Just before assign STOP! STOP = " << STOP << endl;
        STOP = step;
        cout << "Just after assign STOP! (printed in C++)" << endl;
    }
    else if (STOP)
    {   
        throw std::runtime_error("Now the robot is not moving, but stop is not 0.");
    }
}

void start_cartesian_vel_control(franka::Robot *robot)
{
    setDefaultBehavior(*robot);
    pthread_t tids[1];
    cartesian_vel_control_data *args = new cartesian_vel_control_data;
    args->callback = cartesian_vel_callback;
    args->robot = robot;
    cout << "Just Before creating a cartesian vel control thread" << endl;
    pthread_create(&tids[0], NULL, run_cartesian_vel, args);
    cout << "Just after creating a cartesian vel control thread at " << tids[0] << endl;
}

void start_cartesian_pos_control(franka::Robot *robot)
{
    setDefaultBehavior(*robot);
    pthread_t tids[1];
    cartesian_pos_control_data *args = new cartesian_pos_control_data;
    args->callback = cartesian_pos_callback;
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

void start_gripper(franka::Gripper *gripper)
{
    pthread_t tids[1];
    GripperTarget = CurrentGripperState.width;
    pthread_create(&tids[0], NULL, move_gripper, gripper);
}

void set_gripper_motion(double width, double speed = -1.0)
{
    if(width >= CurrentGripperState.max_width)
    {
        cout << "Warnning: the target width " << width << " is too large for the current fingers on the gripper!\n"
             << "The maximum width is " << CurrentGripperState.max_width
             << endl;
        GripperTarget = CurrentGripperState.max_width;
    }
    else
        GripperTarget = width;

    if(speed > 0)
        GripperSpeed = speed;
}

void reset(franka::Robot *robot)
{
    cout << "robot address in reset(): " << robot << endl;
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    RonMoving = true;
    try
    {
        robot->control(motion_generator);
        MotionGenerator motion_generator2(0.5, ResetGoal);
        robot->control(motion_generator2);
        CurrentRobotState = robot->readOnce();
    }
    catch(const franka::ControlException& e)
    {
        handle_exception(robot, e);
    }
    catch(const franka::NetworkException& e)
    {
        handle_exception(robot, e);
    }
    cout << "Reset the robot" << endl;
    RonMoving = false;
}

void update_state(franka::Robot *robot, franka::Gripper *gripper)
{
    cout << "Just before updating the state, robot address = " << robot << endl;
    CurrentRobotState = robot->readOnce();
    CurrentGripperState = gripper->readOnce();
    cout << "Just before updating the state" << endl;
}

void homing(franka::Gripper *gripper)
{
    gripper->homing();
}

double get_gwidth()
{
    return CurrentGripperState.width;
}

int get_Exception_Occur()
{
    return ExceptionOccur;
}

void get_O_T_EE(double* state)
{
    for (int i = 0; i < 16; i++)
        state[i] = CurrentRobotState.O_T_EE_c[i];
}

void get_O_dP_EE_d(double* state)
{
    for (int i = 0; i < 6; i++)
        state[i] = CurrentRobotState.O_dP_EE_d[i];
}

void get_q(double* state)
{
    for (int i = 0; i < 7; i++)
        state[i] = CurrentRobotState.q[i];
}

void get_dq(double* state)
{
    for (int i = 0; i < 7; i++)
        state[i] = CurrentRobotState.dq[i];
}

void set_cartesian_vel(double *action)
{
    for (int i = 0; i < 6; i++)
    {
        CartesianVeloticy[i] = action[i];
    }
}

void set_cartesian_pos(double *pos)
{
    for (int i = 0; i < 16; i++)
    {
        CartesianPosition[i] = pos[i];
    }
}

void set_joint_vel(double *vel)
{
    for (int i = 0; i < 7; i++)
    {
        JointVelocity[i] = vel[i];
    }
}

bool get_RonMoving()
{
    return RonMoving;
}

bool get_GonMoving()
{
    return GonMoving;
}