#ifndef UNICYCLE_KINEMATICS_SIMULATOR_H_
#define UNICYCLE_KINEMATICS_SIMULATOR_H_

/* @libraries */
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <unicycle_model.h>

/* @constants */
#define NAME_OF_THIS_NODE "unicycle_kinematics_simulator"


class unicycle_kinematics_simulator {
  /* @class : unicycle kinematics simulator
   */
  private:
    /* ROS handler */
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;
    ros::Publisher clock_publisher;
    
    /* Parameters from ROS parameter server (yalm file) */
    double dt;
    config_t simulator_config;
    state_t simulator_state;
    input_t simulator_u;

    /* ROS topic callbacks */
    void input_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    unicycle_model* simulator;

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif /* UNICYCLE_KINEMATICS_SIMULATOR_H_ */
