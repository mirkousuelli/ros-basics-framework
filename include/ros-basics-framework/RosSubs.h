/* developed by mirko usuelli
 */
#ifndef ROS_SUBS_H
#define ROS_SUBS_H

#include "RosSyncObjs.h"
#include <map>
#include <vector>
#include <string>

using namespace std;

/* extenting the class Ros Synchronous Objects 
 * in order to maganege a list of subscribers 
 */
template <class T>
class RosSubs : public RosSyncObjs<ros::Subscriber> {
    private:
        map<string, vector<T>> _buffer;

    public:
    /* ---METHODS------------------------------------------------------------------------ */
    RosSubs<T>(void){};

    /* checking existence */
    /*bool contain(string name)
    {
        _topics.contains(name);
    }*/

    /* get object */
    const T& get(string name)
    {
        return _topics.find(name)->second;
    }
    
    /* add new object */
    void add(string name, T obj) 
    {
        _topics.insert(make_pair(name, obj));
        _buffer.insert(make_pair(name, new vector<T>));
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
        _buffer.find(name)->second.push_back(msg);
    }

    /* read and delete the last value */
    T pop(string name)
    {
        return _buffer.find(name)->second.pop_back();
    }

};

#endif /* ROS_SUBS_H */
