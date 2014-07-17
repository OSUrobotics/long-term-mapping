#include "../include/potential_grid.h"
#include "../include/astar.h"
#include "../include/kd_tree.h"
#include "../include/node.h"
#include <unistd.h>


class Test {
public:
	Test();

	void goal_callback(const geometry_msgs::PoseStamped& goal);

private:
	ros::Publisher path_pub_;

	ros::Subscriber goal_sub_;

};

Test::Test(){

	ros::NodeHandle private_nh("~");

	ros::Publisher path_pub_ = private_nh.advertise<nav_msgs::Path>("/tree", 1);
		
	ros::Subscriber goal_sub_ = private_nh.subscribe("/nav_goal", 1, goal_callback);

	ros::spin();
}






int main(int argc, char** argv){
	std::string planner;

	ros::init(argc, argv, "timeglobal_planner");

	


	
	//Show debug output
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}
	

	ROS_DEBUG("node: 1");

	// KDTree t(root);

	ROS_DEBUG("node: 2");

	nav_msgs::Path path;
	path.header.frame_id = "/map";
	path.header.stamp    = ros::Time::now();
	unsigned int micro = 500000;


	for(int i = 0; i < 100; i++){
		KDNode *new_node = new KDNode;

		new_node->val[0] = 10.0 * ((double) rand() / RAND_MAX) - 5.0;
		new_node->val[1] = 10.0 * ((double) rand() / RAND_MAX) - 5.0;
		// new_node->val[2] = 200.0 * ((double) rand() / RAND_MAX);

		// ROS_DEBUG("node: 3");

		t.insert(new_node);

		// ROS_DEBUG("node: 4");

		t.get_display(path);

		// ROS_DEBUG("node: 5");
	// while(true){




	// }
	// ros::param::param<std::string>("/timeglobal_planner/planner", planner, "astar");

	// if(planner == "astar"){
	// 	ROS_DEBUG("Using A*");
	// 	timeglobal_planner::AStar p;
	// }
	// else if(planner == "potential_grid"){
	// 	ROS_DEBUG("Using Potential Grid");
	// 	timeglobal_planner::PotentialGrid p;		
	// }
	}
		usleep(micro);
		path_pub_.publish(path);


	ros::spin();

	return 0;
}
