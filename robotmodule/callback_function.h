#ifndef CALLBACK_FUN_H_
#define CALLBACK_FUN_H_
#include <franka/robot.h>
#include <array>
#include <cmath>
#include "utils.h"


double oteec[16] = {0};
franka::RobotState CurrentState;
double time_ = 0;

std::array<double, 6> ACTION = {0, 0, 0, 0, 0, 0};
std::array<double, 6> last_action = {0};
std::array<double, 6> target_action = {0};
franka::CartesianVelocities velocity_callback(const franka::RobotState &robot_state, franka::Duration period)
{
    constexpr double Epsilon = 1E-4;
    constexpr double Duration = 4e-2;  // For velocity_callback, D=4e-2
    time_ += period.toSec();
    CurrentState = robot_state;
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
    CurrentState = robot_state;
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
    CurrentState = robot_state;
    std::cout << "set oteec. ";
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
    CurrentState = robot_state;
    franka::JointPositions output = {{initial_joint_pos[0] + Amplitude * std::sin(2 * M_PI * time_ / Duration), initial_joint_pos[1],
                                        initial_joint_pos[2], initial_joint_pos[3],
                                        initial_joint_pos[4], initial_joint_pos[5],
                                        initial_joint_pos[6]}};
    if (time_ > Duration)
        return franka::MotionFinished(output);
    
    std::cout << "Move joint for one step at " << time_ << std::endl;
    return output;
}

std::array<double, 7> JointVelocity;
franka::JointVelocities joint_vel_callback(const franka::RobotState& robot_state, franka::Duration period)
{
    constexpr double Amplitude = 0.05;
    constexpr double Duration = 8;
    time_ += period.toSec();
    CurrentState = robot_state;
    double omega = Amplitude * std::sin(2 * M_PI * time_ / Duration);
    franka::JointVelocities output = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
    if (time_ > Duration)
        return franka::MotionFinished(output);
    
    return output;
}

#endif // _CALLBACK_FUN_H_