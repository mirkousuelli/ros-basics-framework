/* developed by mirko usuelli
 */
#include "ros-basics-framework/publisher_class.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

/* -------------------------------------------------------------------------------------------------
 * CALLBACKS METHODS
 * -------------------------------------------------------------------------------------------------
 */

// TODO : add callbacks method if there are any subscribers 

/* -------------------------------------------------------------------------------------------------
 * NODE LIFE CYCLE METHODS
 * -------------------------------------------------------------------------------------------------
 */

/* (0) defining the internal periodic phase */
void publisher_class::_PeriodicTask(void)
{
    // TODO: add all the periodic computation 
    /* ---------------------------------------- */ // <--- down here
    
    std_msgs::Int32 msg1 = read_out("/add1");
    std_msgs::Int32 msg2 = read_out("/add2");

    //msg1.data = 1;
    getPublisher("/add1").publish(msg1);
    ROS_INFO("[%s] sent %d to topic '/add1'", ros::this_node::getName().c_str(), msg1.data);

    //msg2.data = 2;
    getPublisher("/add2").publish(msg2);
    ROS_INFO("[%s] sent %d to topic '/add2'", ros::this_node::getName().c_str(), msg2.data);

    /* ---------------------------------------- */ // <--- until here
}

/* (1) beginning phase */
void publisher_class::Prepare(void)
{
    std_msgs::Int32 msg1;
    std_msgs::Int32 msg2;

    run_period = 1;

    // TODO : add publisher and subscribers to topics
    /* ---------------------------------------- */ // <--- down here
    
    // add1 publisher
    addPublisher("/add1", _handler.advertise<std_msgs::Int32>("/add1", 1));
    ROS_INFO("[%s] added a publisher on topic '/add1'", ros::this_node::getName().c_str());

    // add2 publisher
    addPublisher("/add2", _handler.advertise<std_msgs::Int32>("/add2", 1));
    ROS_INFO("[%s] added a publisher on topic '/add2'", ros::this_node::getName().c_str());

    /* ---------------------------------------- */ // <--- until here

    ROS_INFO("[%s] ready to run", ros::this_node::getName().c_str());

    std::cout << "[" << ros::this_node::getName().c_str() << "] insert value to publish on topic '/add1' : "; 
    std::cin >> msg1.data;
    store_out("/add1", msg1);

    std::cout << "[" << ros::this_node::getName().c_str() << "] insert value to publish on topic '/add2' : ";
    std::cin >> msg2.data;
    store_out("/add2", msg2);
}

/* (2) running phase */
void publisher_class::RunPeriodically(float run_period)
{
    // setting the loop rate
    ros::Rate LoopRate(1.0 / run_period);
    ROS_INFO("[%s] running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), run_period, 1.0 / run_period);

    // entering the loop until external interrupt (e.g. ctrl + C)
    while (ros::ok())
    {
        // running the periodic task
        _PeriodicTask();

        // listening to the ROS framework
        ros::spinOnce();

        // rate delay
        LoopRate.sleep();
    }
}

/* (3) ending phase */
void publisher_class::Shutdown(void)
{
    // Nothing to be done, just shutdown
    ROS_INFO("[%s] shutting down...", ros::this_node::getName().c_str());
}
