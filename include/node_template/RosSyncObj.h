#ifndef ROS_SYNC_OBJ_H
#define ROS_SYNC_OBJ_H

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
  
using namespace std;

class RosSyncObj<T, N>
{
  private: 
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* object : publisher or subscriber */
    vector<T> _objs;

    /* Node state variables */
    vector<N> _payloads;

  public:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* names */
    vector<string> names;

    /* ---METHODS------------------------------------------------------------------------ */

    /* get object index */
    int getIndex(string name)
    {
        // iterator on names
        std::vector<string>::iterator itr = std::find(names.begin(), names.end(), name);
        
        // if the name has been found
        if (itr != names.cend()) 
        {
            // return the distance of the interator from the beginning
            return std::distance(names.begin(), itr);
        }

        // error : not found
        return -1;
    }

    /* get name */
    string getName(int idx) 
    {
        return names[idx];
    }

    /* get object */
    T& getObj(int idx)
    {
        return _objs[idx];
    }

    /* get object payload */
    N& getPayload(int idx)
    {
        return _payloads[idx];
    }
    
    /* add new object */
    void addObj(string name, T obj, N payload) 
    {
        // in all vectors composing the Ros Synchronous Object
        names.push_back(name);
        _objs.push_back(obj);
        _payloads.push_back(payload);
    }
    
    /* delete object */
    void delObj(int idx) 
    {
        // in all vectors composing the Ros Synchronous Object
        names.erase(idx);
        _objs.erase(idx);
        _payloads.erase(idx);
    }

};

#endif /* ROS_SYNC_OBJ_H */