#include "../include/potential_grid.h"
#include "../include/astar.h"
#include "../include/kd_tree.h"


int main(int argc, char** argv){
	std::string planner;

	ros::init(argc, argv, "timeglobal_planner");

	//Show debug output
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}
	
	ros::param::param<std::string>("/timeglobal_planner/planner", planner, "astar");

	if(planner == "astar"){
		ROS_DEBUG("Using A*");
		timeglobal_planner::AStar p;
	}
	else if(planner == "potential_grid"){
		ROS_DEBUG("Using Potential Grid");
		timeglobal_planner::PotentialGrid p;		
	}


	return 0;
}
