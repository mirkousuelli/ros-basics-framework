/* developed by mirko usuelli
 */
#ifndef RBF_SUBS_LIST_H
#define RBF_SUBS_LIST_H

#include "ros-basics-framework/rbf_topics_list.h"
#include "ros/ros.h"
#include <map>
#include <string>

using namespace std;

/* extenting the class Ros Synchronous Objects 
 * in order to maganege a list of subscribers 
 */
template <class T>
class rbf_subs_list : public rbf_topics_list<ros::Subscriber> {
    private:
    /* ---ATTRIBUTE------------------------------------------------------------------------ */
        map<string, T> _buffer;

    public:
    /* ---METHODS------------------------------------------------------------------------ */

        /* get object */
        const ros::Subscriber& get(string name)
        {
            return _topics.find(name)->second;
        }
        
        /* add new object */
        void add(string name, ros::Subscriber obj) 
        {
            _topics.insert(make_pair(name, obj));
            _buffer.insert(make_pair(name, T()));
        }

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

#endif /* RBF_SUBS_LIST_H */