/* developed by mirko usuelli
 */
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "ros/ros.h"
#include "RosNode/RosPubs.h"
#include "RosNode/RosSubs.h"

class RosNode
{
  private: 
    // ---ATTRIBUTES------------------------------------------------------------------------

    /* handler */
    ros::NodeHandle _handler;

    /* Publishers */
    RosPubs _pubs;

    /* Subscribers */
    RosSubs<double> _subs;
    
    /* Parameters from ROS server */
    double _param;

    // ---ABSTRACT METHODS------------------------------------------------------------------- 
    /* Abstract ethods to be developed inside the file "RosNode.cpp" of each node which needs to
     * implement these life-phases-methods in its own directory. This allows a strong 
     * customizable approach in developing the life cycle of a ROS node.
     */

    /* (0) defining the internal periodic phase : task in loop */
    void _PeriodicTask(void); // TODO

  public:
    // ---ATTRIBUTES------------------------------------------------------------------------ 

    /* run period time */
    float run_period;

    // ---FIXED METHODS---------------------------------------------------------------------
    /* Primitive methods already implemented, fixed for each instance of RosNode.
     */

    /* use handler */
    const ros:NodeHandle& useHandler()
    {
      return _handler;
    }

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

    // ---ABSTRACT METHODS------------------------------------------------------------------- 

    /* (1) beginning phase : initialization of connection, publishers, subscribers, ...etc... */
    void Prepare(void); // TODO
    
    /* (2) running phase : taking into account the loop task previously defined in given rate */
    void RunPeriodically(float run_period); // TODO
    
    /* (3) ending phase : shutting down the node*/
    void Shutdown(void); // TODO
};

#endif /* ROS_NODE_H */