/* developed by mirko usuelli
 */
#ifndef ROS_PUBS_H
#define ROS_PUBS_H

#include "ros-basics-framework/RosSyncObjs.h"
#include <map>
#include <string>

using namespace std;

/* extenting the class Ros Synchronous Objects 
 * in order to maganege a list of publishers 
 */
class RosPubs : public RosSyncObjs<ros::Publisher> {
	public:
		/* ---METHODS------------------------------------------------------------------------ */
		//RosPubs(void){};

		/* checking existence */
		/*bool contain(string name)
		{
			_topics.contains(name);
		}*/

		/* get object */
		ros::Publisher& get(string name)
		{
			return _topics.find(name)->second;
		}
		
		/* add new object */
		void add(string name, ros::Publisher obj) 
		{
			_topics.insert(make_pair(name, obj));
		}
		
		/* delete object */
		/*bool del(string name) 
		{
			// checking existence
			if (_topics.contains(name))
			{
				// existing and deleted
				_topics.erase(name);
				return true;
			}

			// not existing item
			return false;
		}*/
};

#endif /* ROS_PUBS_H */
