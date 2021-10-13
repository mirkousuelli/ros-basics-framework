/* developed by mirko usuelli
 */
#ifndef RBF_NODE_H
#define RBF_NODE_H

#include "ros/ros.h"
#include "ros-basics-framework/rbf_topics_list.h"
#include <string>

using namespace std;

template <class T>
class rbf_node
{
  private: 
    // ---ATTRIBUTES------------------------------------------------------------------------

    /* Publishers */
    rbf_topics_list<ros::Publisher, T> _out;

    /* Subscribers */
    rbf_topics_list<ros::Subscriber, T> _in;
  
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
      _in.add(name, sub);
    }

    /* insert publisher */
    void addPublisher(string name, ros::Publisher pub)
    {
      _out.add(name, pub);
    }

    /* subscriber getter */
    const ros::Subscriber& getSubscriber(string name)
    {
      return _in.get(name);
    }

    /* publisher getter */
    const ros::Publisher& getPublisher(string name)
    {
      return _out.get(name);
    }

    /* store a value */
    void store_in(string name, T msg)
    {
      _in.store(name, msg);
    }

    /* store a value */
    void store_out(string name, T msg)
    {
      _out.store(name, msg);
    }

    /* read and delete the last value */
    T read_in(string name)
    {
      return _in.read(name);
    } 

    /* read and delete the last value */
    T read_out(string name)
    {
      return _out.read(name);
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