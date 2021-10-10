/* developed by mirko usuelli
 */
#ifndef ROS_SYNC_OBJ_H
#define ROS_SYNC_OBJ_H

#include <iostream>
#include <string>
#include <map> 

template <class T>
class RosSyncObj
{
  private: 
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* map : < key = topic name ; value = publisher or subscriber instance> */
    map<string, T> _objs;

  public:
    /* ---METHODS------------------------------------------------------------------------ */

    /* checking existence */
    bool contain(string topic)
    {
      _objs.contains(topic);
    }

    /* get object */
    const T& get(string topic)
    {
      return _objs.find(topic);
    }
    
    /* add new object */
    void add(string topic, T obj) 
    {
      _objs.insert(topic, obj);
    }
    
    /* delete object */
    bool del(string topic) 
    {
      // checking existence
      if (_objs.contains(topic))
      {
        // existing and deleted
        _objs.erase(topic);
        return true;
      }

      // not existing item
      return false;
    }
};

#endif /* ROS_SYNC_OBJ_H */