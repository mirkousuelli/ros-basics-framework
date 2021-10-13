/* developed by mirko usuelli
 */
#ifndef RBF_PUBS_LIST_H
#define RBF_PUBS_LIST_H

#include "ros-basics-framework/rbf_topics_list.h"
#include "ros/ros.h"
#include <map>
#include <string>

using namespace std;

/* extenting the class Ros Synchronous Objects 
 * in order to maganege a list of publishers 
 */
class rbf_pubs_list : public rbf_topics_list<ros::Publisher> {
	public:
		/* ---METHODS------------------------------------------------------------------------ */

		/* get object */
		const ros::Publisher& get(string name)
		{
			return _topics.find(name)->second;
		}
		
		/* add new object */
		void add(string name, ros::Publisher obj) 
		{
			_topics.insert(make_pair(name, obj));
		}
};

#endif /* RBF_PUBS_LIST_H */
