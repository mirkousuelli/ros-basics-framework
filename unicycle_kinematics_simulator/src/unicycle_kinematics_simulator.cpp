#include "unicycle_kinematics_simulator/unicycle_kinematics_simulator.h"

#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>

void unicycle_kinematics_simulator::Prepare(void)
{
    /* variables initialization */
    for(int i = 0; i < NUM_CONFIG; i++) {
        simulator_config.push_back(0.0);
    }
    for(int i = 0; i < NUM_STATE; i++) {
        simulator_state.push_back(0.0);
    }
    for(int i = 0; i < NUM_INPUT; i++) {
        simulator_u.push_back(0.0);
    }

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // Model initial state
    FullParamName = ros::this_node::getName()+"/x_0";
    if (false == Handle.getParam(FullParamName, simulator_config[X]))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y_0";
    if (false == Handle.getParam(FullParamName, simulator_config[Y]))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/theta_0";
    if (false == Handle.getParam(FullParamName, simulator_config[THETA]))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Simulator parameters
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    input_subscriber = Handle.subscribe("/input_u", 1, &unicycle_kinematics_simulator::input_MessageCallback, this);
    output_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/output_state", 1);
    clock_publisher  = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    /* Initialize node state */

    // Initialize the simulator
    simulator = new unicycle_model(dt);
    simulator->setConfigValues(simulator_state);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void unicycle_kinematics_simulator::RunPeriodically(void)
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

void unicycle_kinematics_simulator::Shutdown(void)
{
    delete simulator;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void unicycle_kinematics_simulator::input_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    for(int i = 0; i < NUM_INPUT; i++) {
        simulator_u[i] = msg->data[i];
    }

    // storing input commands
    simulator->setInputValues(simulator_u);

    // getting time
    double time;
    simulator->getTime(time);  

    // getting configuration and state
    simulator->getConfigValues(simulator_config);
    simulator->getStateValues(simulator_state);
    
    /* message publishing protocol */
    std_msgs::Float64MultiArray outputMsg;
    outputMsg.data.clear();
    outputMsg.data.push_back(time);

    for (int i = 0; i < NUM_INPUT; i++) {
        outputMsg.data.push_back(simulator_u[i]);
    }
    for (int i = 0; i < NUM_CONFIG; i++) {
        outputMsg.data.push_back(simulator_config[i]);
    }
    for (int i = 0; i < NUM_STATE; i++) {
        outputMsg.data.push_back(simulator_state[i]);
    }

    output_publisher.publish(outputMsg);
}

void unicycle_kinematics_simulator::PeriodicTask(void)
{
    /* Integrate one step ahead */
    double time;
    simulator->getTime(time);
    simulator->updateModel(); 

    /*  Print simulation time every 5 sec */
    if (std::fabs(std::fmod(time,5.0)) < 1.0e-3) {
        // getting current state after 5 seconds    
        simulator->getConfigValues(simulator_config);
        simulator->getStateValues(simulator_state);

        // resume
        std::cout << "TIME -----------------------------------------------------------------" << std::endl;
        ROS_INFO("Simulator time: %d [s]", (int) time);
        std::cout << "INPUT ----------------------------------------------------------------" << std::endl;
        ROS_INFO("Linear velocity: %.2f [m/s]", (float) simulator_u[VEL_LIN]);
        ROS_INFO("Angular velocity: %.2f [deg/s]", (float) simulator_u[VEL_ANG]);
        std::cout << "POSITION -------------------------------------------------------------" << std::endl;
        ROS_INFO("x = %.2f [m]", (float) simulator_config[X]);
        ROS_INFO("y = %.2f [m]", (float) simulator_config[Y]);
        ROS_INFO("theta = %.2f [deg]", (float) simulator_config[THETA]);
        std::cout << "VELOCITY -------------------------------------------------------------" << std::endl;
        ROS_INFO("d_x = %.2f [m/s]", (float) simulator_state[D_X]);
        ROS_INFO("d_y = %.2f [m/s]", (float) simulator_state[D_Y]);
        ROS_INFO("d_theta = %.2f [deg/s]", (float) simulator_state[D_THETA]);
        std::cout << std::endl << std::endl;
    }
}
