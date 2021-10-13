/* developed by mirko usuelli
 */
#ifndef RBF_TOPICS_LIST_H
#define RBF_TOPICS_LIST_H

#include <iostream>
#include <string>
#include <map> 
#include <vector>

using namespace std;

template <class T, class N>
class rbf_topics_list
{
  protected:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* map : < key = topic name ; value = publisher or subscriber instance> */
    map<string, T> _topics;
    map<string, N> _buffer;

  public: 
    /* ---METHODS------------------------------------------------------------------------ */

    /* get object */
    const T& get(string name)
    {
      return _topics.find(name)->second;
    }
      
    /* add new object */
    void add(string name, T obj) 
    {
      _topics.insert(make_pair(name, obj));
      _buffer.insert(make_pair(name, N()));
    }

    /* store a value */
    void store(string name, N msg)
    {
      _buffer.find(name)->second = msg;
    }

    /* read and delete the last value */
    const N read(string name)
    {
      return _buffer.find(name)->second;
    }
};

#endif /* RBF_TOPICS_LIST_H */
