/* developed by mirko usuelli
 */
#include "RosNode/RosNode.h"
#include <std_msgs/Int32.h>

/* -------------------------------------------------------------------------------------------------
 * CALLBACKS METHODS
 * -------------------------------------------------------------------------------------------------
 */

// TODO : add callbacks method if there are any subscribers 

void add1_callback(const std_msgs::Int32::ConstPtr& msg){
    // read message and store information 
    getSubscriber("/add1").push(msg->data);
    ROS_INFO("[Node %s] received %d from topic '/add1'", ros::this_node::getName().c_str(), msg->data);
}

void add2_callback(const std_msgs::Int32::ConstPtr& msg){
    // read message and store information 
    getSubscriber("/add2").push(msg->data);
    ROS_INFO("[Node %s] received %d from topic '/add2'", ros::this_node::getName().c_str(), msg->data);
}

/* -------------------------------------------------------------------------------------------------
 * NODE LIFE CYCLE METHODS
 * -------------------------------------------------------------------------------------------------
 */

/* (0) defining the internal periodic phase */
void RosNode::_PeriodicTask(void)
{
    // TODO: add all the periodic computation 
    /* ---------------------------------------- */ // <--- down here
    
    std_msgs::Int32 sum;

    sum.data = getPublisher("/add1").pop() + getPublisher("/add2").pop();
    getPublisher("/sum").publish(sum);
    ROS_INFO("[Node %s] sent %d to topic '/sum'", ros::this_node::getName().c_str(), sum.data);

    /* ---------------------------------------- */ // <--- until here
}

/* (1) beginning phase */
void RosNode::Prepare(void)
{
    // Retrieve parameters from ROS parameter server
    std::string full_param_name;

    // run_period : from ROS server
    full_param_name = ros::this_node::getName() + "/run_period";
    if (false == useHandler().getParam(full_param_name, run_period))
        ROS_ERROR("[Node %s] unable to retrieve run period %s", ros::this_node::getName().c_str(), full_param_name.c_str());

    // example_parameter : from ROS server
    full_param_name = ros::this_node::getName() + "/_param";
    if (false == useHandler().getParam(full_param_name, _param))
        ROS_ERROR("[Node %s] unable to retrieve parameter %s", ros::this_node::getName().c_str(), full_param_name.c_str());

    // TODO : add publisher and subscribers to topics
    /* ---------------------------------------- */ // <--- down here
    
    // add1 subscriber
    addSubscriber("/add1", useHandler().subscribe("/add1", 1, &RosNode::add1_callback, this));
    ROS_INFO("[Node %s] added a subscriber on topic '/add1'", ros::this_node::getName().c_str());

    // add2 subscriber
    addSubscriber("/add2", useHandler().subscribe("/add2", 1, &RosNode::add2_callback, this));
    ROS_INFO("[Node %s] added a subscriber on topic '/add2'", ros::this_node::getName().c_str());

    // sum publisher
    addPublisher("/sum", useHandler().advertise<std_msgs::Int32>("/sum", 1));
    ROS_INFO("[Node %s] added a publisher on topic '/sum'", ros::this_node::getName().c_str());

    /* ---------------------------------------- */ // <--- until here

    ROS_INFO("[Node %s] ready to run", ros::this_node::getName().c_str());
}

/* (2) running phase */
void RosNode::RunPeriodically(float run_period)
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
void RosNode::Shutdown(void)
{
    // Nothing to be done, just shutdown
    ROS_INFO("[Node %s] shutting down...", ros::this_node::getName().c_str());
}