/* developed by mirko usuelli
 */
#ifndef RBF_TOPICS_LIST_H
#define RBF_TOPICS_LIST_H

#include <iostream>
#include <string>
#include <map> 
#include <vector>

using namespace std;

template <class T>
class rbf_topics_list
{
  protected:
    /* ---ATTRIBUTES--------------------------------------------------------------------- */

    /* map : < key = topic name ; value = publisher or subscriber instance> */
    map<string, T> _topics;

  public: 
    /* ---METHODS------------------------------------------------------------------------ */

    /* get object */
    const T& get(string name);
    
    /* add new object */
    void add(string name, T obj);
};

#endif /* RBF_TOPICS_LIST_H */
