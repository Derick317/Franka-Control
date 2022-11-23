#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <unistd.h>
#include <typeinfo>
#include "examples_common.h"

/*
* This program will do four things:
* 1. move the robot to a suitable joint configuration;
  2. move the robot to an object and grasp it;
  3. move the object to another place.
  4. move the robot to the initial joint configuration;
*
*/

std::array<double, 3> get_xyz(const std::array<double, 16>& O_T_EE)
{
    std::array<double, 3> xyz = {O_T_EE[12], O_T_EE[13], O_T_EE[14]};
    return xyz;
}

void xyz_2_OTEE(std::array<double, 3> xyz, std::array<double, 16>& O_T_EE)
{
    O_T_EE[12] = xyz[0];
    O_T_EE[13] = xyz[1];
    O_T_EE[14] = xyz[2];
}

double dist(double delta_x, double delta_y)
{
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

std::array<double, 3> first_grasp(std::array<double, 3> &begin, std::array<double, 3> &end, double duration, double time)
{
    double h = begin[2] - end[2];
    double d = dist(begin[0] - end[0], begin[1] - end[1]);
    double theta = M_PI_4 * (1 - std::cos(M_PI * time / duration)); // for 0 < time < duration, 0 < theta < pi/2
    double delta_z = h * (std::cos(theta) - 1);
    double delta_xy = d * std::sin(theta);
    std::array<double, 3> position = {
        begin[0] + delta_xy * (end[0] - begin[0]) / d,
        begin[1] + delta_xy * (end[1] - begin[1]) / d,
        begin[2] + delta_z
    };
    return position;
}

std::array<double, 3> second_move(std::array<double, 3> begin, std::array<double, 3> &end, double duration, double time)
{
    double h = end[2] - begin[2];
    double d = dist(begin[0] - end[0], begin[1] - end[1]);
    if (d < 2 * abs(h))
        throw "Bad pose!";
    double radius = (h * h + d * d) / (2 * d);
    double beta = std::asin(h / radius);
    double theta = (M_PI - beta) / 2 * (1 - std::cos(M_PI * time / duration));
    double delta_z = radius * std::sin(theta);
    double delta_xy = radius * (1 - std::cos(theta));
    std::array<double, 3> position = {
        begin[0] + delta_xy * (end[0] - begin[0]) / d,
        begin[1] + delta_xy * (end[1] - begin[1]) / d,
        begin[2] + delta_z
    };
    return position;
}

int main(int argc, char** argv) 
{
    if (argc != 2) 
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    try
    {
        franka::Robot robot(argv[1]);
        franka::Gripper gripper(argv[1]);
        setDefaultBehavior(robot);
        std::cout << robot.readOnce() << std::endl;
        return 0;
        constexpr double duration = 5.0;
        std::array<double, 3> obj_start = {0.617372,-0.185417,0.0652074};
        std::array<double, 3> obj_end = {0.622173,0.280926,0.142912};
        double grasping_width = 0.035;

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot!"
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        gripper.homing();
        std::cout << "Homing once ..." << std::endl;
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        // Do the second and the third thing: move the object
        std::array<double, 16> initial_pose, mid_pose;
        double time = 0.0;
        robot.control([&time, &initial_pose, &mid_pose, &obj_end, &obj_start, &gripper](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
            if (time == 0.0) 
            {
                initial_pose = robot_state.O_T_EE_c;
            }
            time += period.toSec();
            std::array<double, 16> new_pose;
            // Second, grasp the object
            if (time < duration)
            {
                if (time > duration / 2)
                {   
                    std::cout<< "Sleep for 100 ms." << std::endl;
                    sleep(100);
                }
                new_pose = initial_pose;
                std::array<double, 3> initial_xyz = get_xyz(initial_pose);
                xyz_2_OTEE(first_grasp(initial_xyz, obj_start, duration, time), new_pose);
                mid_pose = robot_state.O_T_EE_c;
                std::cout << "Grasping object ... time = " << time << '\r';
            }
            else
                return franka::MotionFinished(robot_state.O_T_EE_c);

            return new_pose;
        });
        if (!gripper.grasp(grasping_width, 0.1, 100)) 
        {
            std::cout << "Failed to grasp object." << std::endl;
            return -1;
        }
        robot.control([&time, &mid_pose, &obj_end, &obj_start, &gripper](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
            time += period.toSec();
            std::array<double, 16> new_pose;
            // Thrid, move the object
            if (time < 2 * duration)
            {
                new_pose = mid_pose;
                xyz_2_OTEE(second_move(get_xyz(mid_pose), obj_end, duration, time - duration), new_pose);
                std::cout << "Moving object ... time = " << time << '\r';
            }
            else
                return franka::MotionFinished(robot_state.O_T_EE_c);

            return new_pose;
        });
        if (!gripper.move(std::min(grasping_width + 0.02, gripper.readOnce().max_width - 0.005), 0.1)) 
        {
            std::cout << "Failed to open the gripper." << std::endl;
            return -1;
        }

        // MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);
        std::cout << std::endl << "Finished motion, shutting down" << std::endl;
    }
    catch (franka::Exception const& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}