/* developed by mirko usuelli
 */
#include "ros-basics-framework/RosNode.h"
#include "ros/ros.h"
#include <std_msgs/Int32.h>

/* -------------------------------------------------------------------------------------------------
 * CALLBACKS METHODS
 * -------------------------------------------------------------------------------------------------
 */

// TODO : add callbacks method if there are any subscribers 

template<class T> void RosNode<T>::add1_callback(T msg)
{
    // read message and store information
    push("/add1", msg);
    ROS_INFO("[Node %s] received %d from topic '/add1'", ros::this_node::getName().c_str(), msg.data);
}

template<class T> void RosNode<T>::add2_callback(T msg)
{
    // read message and store information 
    push("/add2", msg);
    ROS_INFO("[Node %s] received %d from topic '/add2'", ros::this_node::getName().c_str(), msg.data);
}

/* -------------------------------------------------------------------------------------------------
 * NODE LIFE CYCLE METHODS
 * -------------------------------------------------------------------------------------------------
 */

/* (0) defining the internal periodic phase */
template<class T> void RosNode<T>::_PeriodicTask(void)
{
    // TODO: add all the periodic computation 
    /* ---------------------------------------- */ // <--- down here
    
    T sum;

    sum.data = pop("/add1") + pop("/add2");
    getPublisher("/sum").publish(sum);
    ROS_INFO("[Node %s] sent %d to topic '/sum'", ros::this_node::getName().c_str(), sum.data);

    /* ---------------------------------------- */ // <--- until here
}

/* (1) beginning phase */
template<class T> void RosNode<T>::Prepare(void)
{
    // Retrieve parameters from ROS parameter server
    std::string full_param_name;

    // run_period : from ROS server
    full_param_name = ros::this_node::getName() + "/run_period";
    if (false == _handler.getParam(full_param_name, run_period))
        ROS_ERROR("[Node %s] unable to retrieve run period %s", ros::this_node::getName().c_str(), full_param_name.c_str());

    // example_parameter : from ROS server
    full_param_name = ros::this_node::getName() + "/_param";
    if (false == _handler.getParam(full_param_name, _param))
        ROS_ERROR("[Node %s] unable to retrieve parameter %s", ros::this_node::getName().c_str(), full_param_name.c_str());

    // TODO : add publisher and subscribers to topics
    /* ---------------------------------------- */ // <--- down here
    
    // add1 subscriber
    addSubscriber("/add1", _handler.subscribe("/add1", 1, &add1_callback, this));
    ROS_INFO("[Node %s] added a subscriber on topic '/add1'", ros::this_node::getName().c_str());

    // add2 subscriber
    addSubscriber("/add2", _handler.subscribe("/add2", 1, &add2_callback, this));
    ROS_INFO("[Node %s] added a subscriber on topic '/add2'", ros::this_node::getName().c_str());

    // sum publisher
    addPublisher("/sum", _handler.advertise<std_msgs::Int32>("/sum", 1));
    ROS_INFO("[Node %s] added a publisher on topic '/sum'", ros::this_node::getName().c_str());

    /* ---------------------------------------- */ // <--- until here

    ROS_INFO("[Node %s] ready to run", ros::this_node::getName().c_str());
}

/* (2) running phase */
template<class T> void RosNode<T>::RunPeriodically(float run_period)
{
    // setting the loop rate
    ros::Rate LoopRate(1.0 / run_period);
    ROS_INFO("[Node %s] running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), run_period, 1.0 / run_period);

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
template<class T> void RosNode<T>::Shutdown(void)
{
    // Nothing to be done, just shutdown
    ROS_INFO("[Node %s] shutting down...", ros::this_node::getName().c_str());
}
