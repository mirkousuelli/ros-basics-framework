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

    /* Handler */
    ros::NodeHandle _handler;

    /* Publishers */
    RosPubs _pubs;

    /* Subscribers */
    RosSubs _subs;
    
    /* Parameters from ROS server */
    double _param;

    /* ---METHODS------------------------------------------------------------------------ */

    /* Estimator periodic task */
    void _PeriodicTask(void);

  public:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* run period time */
    float run_period;

    /* ---METHODS------------------------------------------------------------------------ */

    /* insert subscriber */
    void addSubscriber(ros::Subscriber sub);

    /* insert publisher */
    void addPublisher(ros::Publisher pub);

    /* subscriber getter */
    const ros::Subscriber& getSubscriber(string name);

    /* publisher getter */
    const ros::Publisher& getPublisher(string name);

    /* delete subscriber */
    int delSubscriber(string name);

    /* delete publisher */
    int delPublisher(string name);

    /* (1) beginning phase */
    void Prepare(void);
    
    /* (2) running phase */
    void RunPeriodically(float period);
    
    /* (3) ending phase */
    void Shutdown(void);
};

#endif /* NODE_TEMPLATE_H_ */