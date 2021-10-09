/* developed by mirko usuelli
 */
#ifndef ROS_SYNC_OBJ_H
#define ROS_SYNC_OBJ_H

#include <iostream>
#include <string>
#include <map> 
  
using namespace std;

template <class T>
class RosSyncObj
{
  private: 
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* map : < key = name ; value = publisher or subscriber instance> */
    map<string, T> _objs;

  public:
    /* ---METHODS------------------------------------------------------------------------ */

    /* checking existence */
    bool contain(string name)
    {
      _objs.contains(name);
    }

    /* get object */
    const T& get(string name)
    {
      return _objs.find(name);
    }
    
    /* add new object */
    void add(string name, T obj) 
    {
      _objs.insert(name, obj);
    }
    
    /* delete object */
    bool del(string name) 
    {
      // checking existence
      if (_objs.contains(name))
      {
        // existing and deleted
        _objs.erase(name);
        return true;
      }

      // not existing item
      return false;
    }
};

#endif /* ROS_SYNC_OBJ_H */