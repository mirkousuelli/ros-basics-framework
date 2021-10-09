#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <vector>
  
using namespace std;

class RosNode
{
  private: 
    /* ---ATTRIBUTES--------------------------------------------------------------------- */
    /* Handler */
    ros::NodeHandle _handler;

    /* Publishers */
    vector<ros::Publisher> _pubs;

    /* Subscribers */
    vector<ros::Subscriber> _subs;
    
    /* Parameters from ROS parameter server */
    double _param;

    /* ---METHODS------------------------------------------------------------------------ */
    /* ROS topic callbacks */
    void inTopic_MessageCallback(const std_msgs::Float64::ConstPtr& msg);

    /* Estimator periodic task */
    void PeriodicTask(void);

  public:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */
    float run_period;

    /* ---METHODS------------------------------------------------------------------------ */
    void Prepare(void);
    
    void RunPeriodically(float period);
    
    void Shutdown(void);

};

#endif /* NODE_TEMPLATE_H_ */
