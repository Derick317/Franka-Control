#include <cmath>
#include <ctime>
#include <iostream>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <unistd.h>
#include "examples_common.h"

double GripperTarget = 0.05, GripperSpeed = 0.01;

void *move_gripper(void *args)
{
    franka::Gripper *gripper = (franka::Gripper *)args;
    double start_time, end_time;
    std::cout << "Just before gripper moving, target width = " << GripperTarget << ", speed = " << GripperSpeed << std::endl;
    start_time = std::clock();
    try
    {
        gripper->move(GripperTarget, GripperSpeed);
    }
    catch (const franka::Exception &e)
    {
        std::cout << "Get an error: " << e.what() << std::endl;
        return NULL;
    }
    end_time = std::clock();
    std::cout << "Just after gripper moving. The movement took " << (end_time - start_time) / 10000 << " s." << std::endl;
    return nullptr;
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
        double start_time, end_time;
        franka::Gripper gripper(argv[1]);
        gripper.homing();
        pthread_t tids[1];
        pthread_create(&tids[0], NULL, move_gripper, &gripper);
        sleep(4);
        std::cout << "Complete the first movement" << std::endl;
        GripperTarget = 0.01;
        // start_time = std::clock();
        // pthread_create(&tids[0], NULL, move_gripper, &gripper);
        gripper.move(0.01, 0.01);
        sleep(10);
        std::cout << "Just before stopping the gripper" << std::endl;
        gripper.move(0.06, 0.01);
        // bool successful;
        // successful = gripper.stop();
        end_time = std::clock();
        // std::cout << "Just after stopping the gripper. " << (successful? "Success": "Fail") << " to stop the gripper. "
        //           << "The movement took " << (end_time - start_time) / 10000 << " s." << std::endl;
    }
    catch (franka::Exception const& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
    std::cout << "Finished!" << std::endl;
    return 0;
}