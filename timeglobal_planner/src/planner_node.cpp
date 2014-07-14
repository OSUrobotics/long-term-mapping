// #include "../include/dijkstra.h"
#include "../include/astar.h"



int main(int argc, char** argv){
	ros::init(argc, argv, "timeglobal_planner");
	
	//Show debug output
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	// timeglobal_planner::Dijkstra p;
	timeglobal_planner::AStar p;

	ros::spin();

	return 0;
}
