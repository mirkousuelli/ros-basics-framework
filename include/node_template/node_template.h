#ifndef NODE_TEMPLATE_H_
#define NODE_TEMPLATE_H_

#include "ros/ros.h"

#include <std_msgs/Float64.h>


#define NAME_OF_THIS_NODE "node_template"


class node_template
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber inTopic_subscriber;
    ros::Publisher outTopic_publisher;
    
    /* Parameters from ROS parameter server */
    double example_parameter;

    /* ROS topic callbacks */
    void inTopic_MessageCallback(const std_msgs::Float64::ConstPtr& msg);

    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double in_data;

  public:
    float RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* NODE_TEMPLATE_H_ */
