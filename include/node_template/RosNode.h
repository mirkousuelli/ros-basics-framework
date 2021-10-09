#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "ros/ros.h"
#include "node_template/RosPubs.h"
#include "node_template/RosSubs.h"
  
using namespace std;

class RosNode
{
  private: 
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* Publishers */
    RosPubs _pubs;

    /* Subscribers */
    RosSubs _subs;
    
    /* Parameters from ROS server */
    double _param;

  public:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* handler */
    ros::NodeHandle handler;

    /* run period time */
    float run_period;

    /* ---FIXED METHODS--------------------------------------------------------------------- */

    /* insert subscriber */
    void addSubscriber(string name, ros::Subscriber sub)
    {
      _subs.add(name, sub);
      ROS_INFO("Node %s has added subscriber %s.", ros::this_node::getName().c_str(), name);
    }

    /* insert publisher */
    void addPublisher(string name, ros::Publisher pub)
    {
      _pubs.add(name, pub);
      ROS_INFO("Node %s has added publisher %s.", ros::this_node::getName().c_str(), name);
    }

    /* subscriber getter */
    const ros::Subscriber& getSubscriber(string name)
    {
      return _subs.get(name);
    }

    /* publisher getter */
    const ros::Publisher& getPublisher(string name)
    {
      return _pubs.get(name);
    }

    /* delete subscriber */
    int delSubscriber(string name) {
      if(_subs.del(name))
        ROS_INFO("Node %s has deleted subscriber %s.", ros::this_node::getName().c_str(), name);
      ROS_ERROR("Failed! Node %s has not found subscriber %s to be deleted.", ros::this_node::getName().c_str(), name);
    }

    /* delete publisher */
    int delPublisher(string name) 
    {
      if(_pubs.del(name))
        ROS_INFO("Node %s has deleted publisher %s.", ros::this_node::getName().c_str(), name);
      ROS_ERROR("Failed! Node %s has not found publisher %s to be deleted.", ros::this_node::getName().c_str(), name);
    }

    /* ---ABSTRACT METHODS--------------------------------------------------------------------- */

    /* (0) defining the internal periodic phase */
    void PeriodicTask(void);

    /* (1) beginning phase */
    void Prepare(void);
    
    /* (2) running phase */
    void RunPeriodically(float period);
    
    /* (3) ending phase */
    void Shutdown(void);
};

#endif /* NODE_TEMPLATE_H_ */