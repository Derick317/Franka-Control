#ifndef ROBOTMODULE_H_
#define ROBOTMODULE_H_
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

bool RonMoving = false; // Whether a control or motion generator loop of robot is active
bool GonMoving = false; // Whether a control or motion generator loop of gripper is active
double GripperTarget = 0;
double GripperSpeed = 0.1;
// {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}
std::array<double, 7> ResetGoal = {{-0.751971, -0.0259919, 0.388233, -2.04774, -0.487198, 3.66707, -0.386603}};

extern "C"
{
    void new_robot(char fci_ip[], void **address);
    
    void new_gripper(char fci_ip[], void **address);

    void stop_robot(long);

    void start_cartesian_pos_control(franka::Robot *robot);

    void start_cartesian_vel_control(franka::Robot *robot);

    void start_joint_pos_control(franka::Robot *robot);

    void start_joint_vel_control(franka::Robot *robot);

    void start_gripper(franka::Gripper *gripper);

    void reset(franka::Robot *robot);

    void homing(franka::Gripper *gripper);

    void set_gripper_motion(double width, double speed);

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