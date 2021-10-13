/* developed by mirko usuelli
 */
#ifndef RBF_NODE_H
#define RBF_NODE_H

#include "ros/ros.h"
#include "ros-basics-framework/rbf_pubs_list.h"
#include "ros-basics-framework/rbf_subs_list.h"
#include <string>

using namespace std;

template <class T>
class rbf_node
{
  private: 
    // ---ATTRIBUTES------------------------------------------------------------------------

    /* Publishers */
    rbf_pubs_list _pubs;

    /* Subscribers */
    rbf_subs_list<T> _subs;
  
  protected:
    /* handler */
    ros::NodeHandle _handler;

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

    /* insert subscriber */
    void addSubscriber(string name, ros::Subscriber sub)
    {
      _subs.add(name, sub);
    }

    /* insert publisher */
    void addPublisher(string name, ros::Publisher pub)
    {
      _pubs.add(name, pub);
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

    /* store a value */
    void push(string name, T msg)
    {
      _subs.push(name, msg);
    }

    /* read and delete the last value */
    T pop(string name)
    {
      return _subs.pop(name);
    } 

    // ---ABSTRACT METHODS------------------------------------------------------------------- 

    /* (1) beginning phase : initialization of connection, publishers, subscribers, ...etc... */
    void Prepare(void); // TODO
    
    /* (2) running phase : taking into account the loop task previously defined in given rate */
    void RunPeriodically(float run_period); // TODO
    
    /* (3) ending phase : shutting down the node*/
    void Shutdown(void); // TODO
};

#endif /* RBF_NODE_H */