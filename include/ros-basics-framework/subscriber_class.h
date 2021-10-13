/* developed by mirko usuelli
 */
#ifndef SUBSCRIBER_CLASS_H
#define SUBSCRIBER_CLASS_H

#include "ros-basics-framework/rbf_node.h"
#include "std_msgs/Int32.h"

using namespace std;

/* extenting the class Ros Synchronous Objects 
 * in order to maganege a list of publishers 
 */
class subscriber_class : public rbf_node<std_msgs::Int32> 
{
    private:
        // ---CALLBACKS-------------------------------------------------------------------
        void _add1_callback(std_msgs::Int32 msg); // TODO
        void _add2_callback(std_msgs::Int32 msg); // TODO

        /* (0) defining the internal periodic phase : task in loop */
        void _PeriodicTask(void); // TODO
        
    public:
        /* (1) beginning phase : initialization of connection, publishers, subscribers, ...etc... */
        void Prepare(void); // TODO
        
        /* (2) running phase : taking into account the loop task previously defined in given rate */
        void RunPeriodically(float run_period); // TODO
        
        /* (3) ending phase : shutting down the node*/
        void Shutdown(void); // TODO  
};

#endif /* SUBSCRIBER_CLASS_H */
