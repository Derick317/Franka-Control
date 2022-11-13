#ifndef ROBOTMODULE_H_
#define ROBOTMODULE_H_
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

bool RonMoving = false; // Whether a control or motion generator loop of robot is active
bool GonMoving = false; // Whether a control or motion generator loop of gripper is active
double GripperTarget = 0;
double GripperSpeed = 0.1;
std::array<double, 7> RobotResetGoal = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
double GripperResetGoal = 0.02;


extern "C"
{
    void new_robot(char fci_ip[], void **address, double * reset_goal);
    
    void new_gripper(char fci_ip[], void **address, double reset_goal);

    void stop_robot(long);

    void stop_gripper(franka::Gripper *gripper);

    void start_cartesian_pos_control(franka::Robot *robot, franka::Gripper *gripper);

    void start_cartesian_vel_control(franka::Robot *robot, franka::Gripper *gripper);

    void start_joint_pos_control(franka::Robot *robot, franka::Gripper *gripper);

    void start_joint_vel_control(franka::Robot *robot, franka::Gripper *gripper);

    /**
    * Moves the gripper fingers to a specified width.
    *
    * @param[in] width Intended opening width. [m]
    * @param[in] speed Closing speed. [m/s]
    **/
    void start_gripper(franka::Gripper *gripper, double width, double speed);

    void reset(franka::Robot *robot, franka::Gripper *gripper, double *robot_reset_goal, double gripper_reset_width);

    void homing(franka::Gripper *gripper);

    // void set_gripper_motion(double width, double speed);

    void update_state(franka::Robot *robot, franka::Gripper *gripper);
    
    void set_cartesian_vel(double *);

    void set_cartesian_pos(double *pos);

    void set_joint_vel(double *vel);

    double get_gwidth();

    void get_O_T_EE(double *state);

    void get_O_dP_EE_d(double *state);

    void get_q(double *state);

    void get_dq(double *state);

    int get_Exception_Occur();

    bool get_RonMoving();

    bool get_GonMoving();
}

#endif // ROBOTMODULE_H_