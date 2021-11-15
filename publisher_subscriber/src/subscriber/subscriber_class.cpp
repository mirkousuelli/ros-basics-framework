/* developed by mirko usuelli
 */
#include "ros-basics-framework/subscriber_class.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"

/* -------------------------------------------------------------------------------------------------
 * CALLBACKS METHODS
 * -------------------------------------------------------------------------------------------------
 */

// TODO : add callbacks method if there are any subscribers 

void subscriber_class::_add1_callback(std_msgs::Int32 msg)
{
    // read message and store information
    store_in("/add1", msg);
    ROS_INFO("[%s] received %d from topic '/add1'", ros::this_node::getName().c_str(), msg.data);
}

void subscriber_class::_add2_callback(std_msgs::Int32 msg)
{
    // read message and store information 
    store_in("/add2", msg);
    ROS_INFO("[%s] received %d from topic '/add2'", ros::this_node::getName().c_str(), msg.data);
}

/* -------------------------------------------------------------------------------------------------
 * NODE LIFE CYCLE METHODS
 * -------------------------------------------------------------------------------------------------
 */

/* (0) defining the internal periodic phase */
void subscriber_class::_PeriodicTask(void)
{
    // TODO: add all the periodic computation 
    /* ---------------------------------------- */ // <--- down here
    
    std_msgs::Int32 sum;

    sum.data = read_in("/add1").data + read_in("/add2").data;
    store_out("/sum", sum);
    getPublisher("/sum").publish(read_out("/sum"));
    ROS_INFO("[%s] sent %d to topic '/sum'", ros::this_node::getName().c_str(), sum.data);

    /* ---------------------------------------- */ // <--- until here
}

/* (1) beginning phase */
void subscriber_class::Prepare(void)
{
    // Retrieve parameters from ROS parameter server
    /*std::string full_param_name;

    // run_period : from ROS server
    full_param_name = ros::this_node::getName() + "/run_period";
    if (false == _handler.getParam(full_param_name, run_period))
        ROS_ERROR("[Node %s] unable to retrieve run period %s", ros::this_node::getName().c_str(), full_param_name.c_str());

    // example_parameter : from ROS server
    full_param_name = ros::this_node::getName() + "/_param";
    if (false == _handler.getParam(full_param_name, _param))
        ROS_ERROR("[Node %s] unable to retrieve parameter %s", ros::this_node::getName().c_str(), full_param_name.c_str());
    */
    run_period = 1;
    // TODO : add publisher and subscribers to topics
    /* ---------------------------------------- */ // <--- down here
    
    // add1 subscriber
    addSubscriber("/add1", _handler.subscribe("/add1", 1, &subscriber_class::_add1_callback, this));
    ROS_INFO("[%s] added a subscriber on topic '/add1'", ros::this_node::getName().c_str());

    // add2 subscriber
    addSubscriber("/add2", _handler.subscribe("/add2", 1, &subscriber_class::_add2_callback, this));
    ROS_INFO("[%s] added a subscriber on topic '/add2'", ros::this_node::getName().c_str());

    // sum publisher
    addPublisher("/sum", _handler.advertise<std_msgs::Int32>("/sum", 1));
    ROS_INFO("[%s] added a publisher on topic '/sum'", ros::this_node::getName().c_str());

    /* ---------------------------------------- */ // <--- until here

    ROS_INFO("[%s] ready to run", ros::this_node::getName().c_str());
}

/* (2) running phase */
void subscriber_class::RunPeriodically(float run_period)
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
void subscriber_class::Shutdown(void)
{
    // Nothing to be done, just shutdown
    ROS_INFO("[%s] shutting down...", ros::this_node::getName().c_str());
}
