#include "unicycle_kinematics_simulator/unicycle_kinematics_tester.h"

#include <std_msgs/Float64MultiArray.h>

void unicycle_kinematics_tester::Prepare(void)
{
    // initialize variables
    for(int i = 0; i < NUM_CONFIG; i++) {
        tester_config.push_back(0.0);
    }
    for(int i = 0; i < NUM_STATE; i++) {
        tester_state.push_back(0.0);
    }
    for(int i = 0; i < NUM_INPUT; i++) {
        tester_u.push_back(0.0);
    }

    /* ROS topics */
    input_subscriber = Handle.subscribe("/output_state", 1, &unicycle_kinematics_tester::input_MessageCallback, this);
    output_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/input_u", 1);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void unicycle_kinematics_tester::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        usleep(1000);
    }
}

void unicycle_kinematics_tester::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void unicycle_kinematics_tester::input_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    /* Read message and store information */
    time = msg->data[0];

    for (int i = 1; i <= NUM_INPUT; i++) {
        tester_u[i - 1] = msg->data[i];
    }
    for (int i = 1 + NUM_INPUT; i <= NUM_INPUT + NUM_CONFIG; i++) {
        tester_config[i - (1 + NUM_INPUT)] = msg->data[i];
    }
    for (int i = NUM_INPUT + NUM_CONFIG + 1; i <= NUM_CONFIG + NUM_INPUT + NUM_STATE; i++) {
        tester_state[i - (NUM_INPUT + NUM_CONFIG + 1)] = msg->data[i];
    }

    std::cout << "TIME -----------------------------------------------------------------" << std::endl;
    ROS_INFO("Simulator time: %d [s]", (int) time);
    std::cout << "INPUT ----------------------------------------------------------------" << std::endl;
    ROS_INFO("Linear velocity: %.2f [m/s]", (float) tester_u[VEL_LIN]);
    ROS_INFO("Angular velocity: %.2f [deg/s]", (float) tester_u[VEL_ANG]);
    std::cout << "CONFIGURATION -------------------------------------------------------------" << std::endl;
    ROS_INFO("x = %.2f [m]", (float) tester_config[X]);
    ROS_INFO("y = %.2f [m]", (float) tester_config[Y]);
    ROS_INFO("theta = %.2f [deg]", (float) tester_config[THETA]);
    std::cout << "STATE -------------------------------------------------------------" << std::endl;
    ROS_INFO("d_x = %.2f [m/s]", (float) tester_state[D_X]);
    ROS_INFO("d_y = %.2f [m/s]", (float) tester_state[D_Y]);
    ROS_INFO("d_theta = %.2f [deg/s]", (float) tester_state[D_THETA]);
    std::cout << std::endl;
}

void unicycle_kinematics_tester::PeriodicTask(void)
{
    std::cout << "Linear velocity [m/s] : ";
    std::cin >> tester_u[VEL_LIN];
    std::cout << "Angular velocity [deg/s] : ";
    std::cin >> tester_u[VEL_ANG];
    std::cout << "----------------------------------------------------------------------";
    std::cout << std::endl << std::endl;

    /* Publish inputs */
    std_msgs::Float64MultiArray outputMsg;
    outputMsg.data.clear();

    for (int i = 0; i < NUM_INPUT; i++) {
        outputMsg.data.push_back(tester_u[i]);
    }

    output_publisher.publish(outputMsg);
}