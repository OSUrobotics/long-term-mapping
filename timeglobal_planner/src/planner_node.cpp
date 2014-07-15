#include "../include/potential_grid.h"
#include "../include/astar.h"



int main(int argc, char** argv){
	std::string planner;

	ros::init(argc, argv, "timeglobal_planner");
	
	//Show debug output
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	ros::param::param<std::string>("planner", planner, "astar");

	if(planner == "astar"){
		timeglobal_planner::AStar p;
	}
	else if(planner == "potential_grid"){
		timeglobal_planner::PotentialGrid p;		
	}

	ros::spin();

	return 0;
}
