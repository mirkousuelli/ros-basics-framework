/* developed by mirko usuelli
 */
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "ros/ros.h"
//#include "ros-basics-framework/RosPubs.h"
//#include "ros-basics-framework/RosSubs.h"
#include "std_msgs/Int32.h"
#include <string>
#include <iostream>
#include <string>
#include <map> 
#include <vector>

using namespace std;


template <class T>
class RosSyncObjs
{
  protected:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* map : < key = topic name ; value = publisher or subscriber instance> */
    map<string, T> _topics;

  public: 
    /* ---METHODS------------------------------------------------------------------------ */
    //RosSyncObjs<T>(void){};

    /* checking existence */
    //bool contain(string name);

    /* get object */
    const T& get(string name);
    
    /* add new object */
    void add(string name, T obj);
    
    /* delete object */
    //bool del(string name);
};

template <class T>
class RosSubs : public RosSyncObjs<ros::Subscriber> {
    private:
        map<string, T> _buffer;

    public:
    /* ---METHODS------------------------------------------------------------------------ */
    //RosSubs<T>(void){};

    /* checking existence */
    /*bool contain(string name)
    {
        _topics.contains(name);
    }*/

    /* get object */
    const ros::Subscriber& get(string name)
    {
        return _topics.find(name)->second;
    }
    
    /* add new object */
    void add(string name, ros::Subscriber obj) 
    {
        _topics.insert(make_pair(name, obj));
        //_buffer.insert(make_pair(name, new vector<T>));
        _buffer.insert(make_pair(name, T()));
    }
    
    /* delete object */
    /*bool del(string name) 
    {
        // checking existence
        if (_topics.contains(name))
        {
        // existing and deleted
        _topics.erase(name);
        _buffer.erase(name);
        return true;
        }

        // not existing item
        return false;
    }*/

    /* store a value */
    void push(string name, T msg)
    {
        _buffer.find(name)->second = msg;
    }

    /* read and delete the last value */
    const T pop(string name)
    {
      return _buffer.find(name)->second;
    }
};

class RosPubs : public RosSyncObjs<ros::Publisher> {
	public:
		/* ---METHODS------------------------------------------------------------------------ */
		//RosPubs(void){};

		/* checking existence */
		/*bool contain(string name)
		{
			_topics.contains(name);
		}*/

		/* get object */
		const ros::Publisher& get(string name)
		{
			return _topics.find(name)->second;
		}
		
		/* add new object */
		void add(string name, ros::Publisher obj) 
		{
			_topics.insert(make_pair(name, obj));
		}
		
		/* delete object */
		/*bool del(string name) 
		{
			// checking existence
			if (_topics.contains(name))
			{
				// existing and deleted
				_topics.erase(name);
				return true;
			}

			// not existing item
			return false;
		}*/
};

class RosNode
{
  private: 
    // ---ATTRIBUTES------------------------------------------------------------------------

    /* handler */
    ros::NodeHandle _handler;

    /* Publishers */
    RosPubs _pubs;

    /* Subscribers */
    RosSubs<std_msgs::Int32> _subs;
    
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

    //RosNode(void){};

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
    /*int delSubscriber(string name) {
      if(_subs.del(name))
        ROS_INFO("Node %s has deleted subscriber %s.", ros::this_node::getName().c_str(), name);
      ROS_ERROR("Failed! Node %s has not found subscriber %s to be deleted.", ros::this_node::getName().c_str(), name);
    }*/

    /* delete publisher */
    /*int delPublisher(string name) 
    {
      if(_pubs.del(name))
        ROS_INFO("Node %s has deleted publisher %s.", ros::this_node::getName().c_str(), name);
      ROS_ERROR("Failed! Node %s has not found publisher %s to be deleted.", ros::this_node::getName().c_str(), name);
    }*/

    /* store a value */
    void push(string name, std_msgs::Int32 msg)
    {
      _subs.push(name, msg);
    }

    /* read and delete the last value */
    std_msgs::Int32 pop(string name)
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

    // ---CALLBACKS------------------------------------------------------------------- 
    void add1_callback(std_msgs::Int32 msg); // TODO
    void add2_callback(std_msgs::Int32 msg); // TODO
};

#endif /* ROS_NODE_H */