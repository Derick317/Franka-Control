#ifndef CALLBACK_FUN_H_
#define CALLBACK_FUN_H_
#include <franka/robot.h>
#include <array>
#include <exception>
#include <cmath>
#include "utils.h"

franka::RobotState CurrentRobotState;
franka::GripperState CurrentGripperState;
// pthread_t tids[2]; // Robot uses tids[0], gripper uses tids[1]
double time_ = 0;
long STOP = 0; // a singal to stop the robot motion

std::array<double, 6> CartesianVeloticy = {0, 0, 0, 0, 0, 0};
std::array<double, 6> last_cartesian_vel = {0};
std::array<double, 6> target_cartesian_vel = {0};
franka::CartesianVelocities cartesian_vel_callback(const franka::RobotState &robot_state, franka::Duration period)
{
    constexpr double Epsilon = 1E-4;
    constexpr double Duration = 4e-2;  // For velocity_callback, D=4e-2
    time_ += period.toSec();
    CurrentRobotState = robot_state;
    if (time_ > Duration)
    {
        time_ = 0;
        if (STOP) 
        {   
            if (STOP <= 1)
                throw "Bad STOP value!s";
            STOP--;
            for(int i = 0; i < 6; i++)
                CartesianVeloticy[i] = 0;
        }
        for (int i = 0; i < 6; i++){
            last_cartesian_vel[i] = target_cartesian_vel[i];
            target_cartesian_vel[i] = CartesianVeloticy[i];
        }
    }
    std::array<double, 6> car_vel_c = {0, 0, 0, 0, 0, 0};
    for(int i = 0; i < 6; i++)
    {
        car_vel_c[i] = point_of_division(last_cartesian_vel[i], target_cartesian_vel[i], time_ / Duration);
    }
    franka::CartesianVelocities output(car_vel_c);
    if (1 == STOP)
    {
        STOP--; // STOP equals 0
        return franka::MotionFinished(output);
    }
    return output;
}


std::array<double, 16> initial_position;
std::array<double, 3> target_xyz;
franka::CartesianPose test_cartesian_pos(const franka::RobotState& robot_state, franka::Duration period)
{
    constexpr double Amplitude = 0.002;
    constexpr double Duration = 10;
    time_ += period.toSec();
    if (0.0 == time_)
    {
        initial_position = robot_state.O_T_EE_c;
        std::cout << "Got initial state." << std::endl;
    }
    CurrentRobotState = robot_state;
    std::array<double, 16> output = initial_position;
    output[14] += Amplitude * std::sin(2 * M_PI * time_ / Duration);
    if (time_ > Duration)
        return franka::MotionFinished(output);
    
    std::cout << "Move for one step at " << time_ << std::endl;
    return output;
}


franka::CartesianPose goto_position(const franka::RobotState& robot_state, franka::Duration period)
{
    constexpr double Duration = 5;
    time_ += period.toSec();
    if (0.0 == time_)
    {
        initial_position = robot_state.O_T_EE_c;
        std::cout << "Got initial state. Target xyz:" << target_xyz[0] << target_xyz[1] << std::endl;
    }
    CurrentRobotState = robot_state;
    if (time_ < Duration)
    {
        std::array<double, 16> output;
        output = initial_position;
        std::array<double, 3> begin = get_xyz(initial_position), end = target_xyz;
        double h = begin[2] - end[2];
        double d = dist(begin[0] - end[0], begin[1] - end[1]);
        double theta = M_PI_4 * (1 - std::cos(M_PI * time_ / Duration)); // for 0 < time < duration, 0 < theta < pi/2
        double delta_z = h * (std::cos(theta) - 1);
        double delta_xy = d * std::sin(theta);
        std::array<double, 3> new_xyz = {
            begin[0] + delta_xy * (end[0] - begin[0]) / d,
            begin[1] + delta_xy * (end[1] - begin[1]) / d,
            begin[2] + delta_z
        };
        xyz_2_OTEE(new_xyz, output);
        std::cout << "Move for one step at " << time_ << std::endl;
        return output;
    }
    else
    {
        std::cout << "Finish moving" << std::endl;
        return franka::MotionFinished(robot_state.O_T_EE_c);
    }
}


std::array<double, 7> JointPosition;
std::array<double, 7> initial_joint_pos;
franka::JointPositions joint_pos_callback(const franka::RobotState& robot_state, franka::Duration period)
{
    constexpr double Amplitude = 0.2;
    constexpr double Duration = 8;
    time_ += period.toSec();
    if (0.0 == time_)
    {
        initial_joint_pos = robot_state.q_d;
        std::cout << "Got initial state." << std::endl;
    }
    CurrentRobotState = robot_state;
    franka::JointPositions output = {{initial_joint_pos[0] + Amplitude * std::sin(2 * M_PI * time_ / Duration), initial_joint_pos[1],
                                        initial_joint_pos[2], initial_joint_pos[3],
                                        initial_joint_pos[4], initial_joint_pos[5],
                                        initial_joint_pos[6]}};
    if (time_ > Duration)
        return franka::MotionFinished(output);
    
    std::cout << "Move joint for one step at " << time_ << std::endl;
    return output;
}

std::array<double, 7> JointVelocity = {0.0};
std::array<double, 7> last_joint_vel = {0.0};
std::array<double, 7> target_joint_vel = {0.0};
franka::JointVelocities joint_vel_callback(const franka::RobotState& robot_state, franka::Duration period)
{
    constexpr double Duration = 1e-1;  // For velocity_callback, D=4e-2
    time_ += period.toSec();
    CurrentRobotState = robot_state;
    if (time_ > Duration)
    {
        time_ = 0;
        if (STOP) 
        {   
            // std::cout << "STOP = " << STOP << ", velocity:";
            // for (int j = 0; j < 7; j++)
            //     std::cout << ' ' << robot_state.dq[j];
            // std::cout << std::endl;
            if (STOP <= 1)
                throw "Bad STOP value!s";
            STOP--;
            for(int i = 0; i < 7; i++)
                JointVelocity[i] = 0;
        }
        for (int i = 0; i < 7; i++){
            last_joint_vel[i] = target_joint_vel[i];
            target_joint_vel[i] = JointVelocity[i];
        }
    }
    franka::JointVelocities output = {{last_joint_vel[0] + (target_joint_vel[0] - last_joint_vel[0]) * time_ / Duration,
                                        last_joint_vel[1] + (target_joint_vel[1] - last_joint_vel[1]) * time_ / Duration,
                                        last_joint_vel[2] + (target_joint_vel[2] - last_joint_vel[2]) * time_ / Duration,
                                        last_joint_vel[3] + (target_joint_vel[3] - last_joint_vel[3]) * time_ / Duration,
                                        last_joint_vel[4] + (target_joint_vel[4] - last_joint_vel[4]) * time_ / Duration,
                                        last_joint_vel[5] + (target_joint_vel[5] - last_joint_vel[5]) * time_ / Duration,
                                        last_joint_vel[6] + (target_joint_vel[6] - last_joint_vel[6]) * time_ / Duration}};
    
    if (1 == STOP)
    {
        STOP--; // STOP equals 0
        return franka::MotionFinished(output);
    }
    
    return output;
}

#endif // _CALLBACK_FUN_H_