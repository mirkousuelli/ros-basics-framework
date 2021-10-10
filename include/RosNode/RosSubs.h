/* developed by mirko usuelli
 */
#ifndef ROS_SUBS_H
#define ROS_SUBS_H

#include "RosNode/RosSyncObj.h"
#include <vector>

/* extenting the class Ros Synchronous Objects 
 * in order to maganege a list of subscribers 
 */
template <class T>
class RosSubs : public RosSyncObj<ros::Subscriber> {
    private:
        vector<T> _buffer;

    public:
        void push(T datum) 
        {
            _buffer.push_back(datum);
        }

        T pop(void) 
        {
            return _buffer.pop_back(datum);
        }
};

#endif /* ROS_SUBS_H */
