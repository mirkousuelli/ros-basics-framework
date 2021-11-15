#ifndef UNICYCLE_KINEMATICS_TESTER_H_
#define UNICYCLE_KINEMATICS_TESTER_H_

/* @libraries */
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <unicycle_model.h>

/* @constants */
#define NAME_OF_THIS_NODE "unicycle_kinematics_tester"

class unicycle_kinematics_tester {
  /* @class : unicycle kinematics tester
   */
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;
    ros::Publisher clock_publisher;
    
    /* Parameters from ROS parameter server (yalm file) */
    config_t tester_config;
    state_t tester_state;
    input_t tester_u;
    double time;

    /* ROS topic callbacks */
    void input_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif /* UNICYCLE_KINEMATICS_TESTER_H_ */
