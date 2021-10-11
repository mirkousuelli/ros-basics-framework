/* developed by mirko usuelli
 */
#ifndef ROS_SYNC_OBJS_H
#define ROS_SYNC_OBJS_H

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
    RosSyncObjs<T>(void){};

    /* checking existence */
    //bool contain(string name);

    /* get object */
    const T& get(string name);
    
    /* add new object */
    void add(string name, T obj);
    
    /* delete object */
    //bool del(string name);
};

#endif /* ROS_SYNC_OBJS_H */
