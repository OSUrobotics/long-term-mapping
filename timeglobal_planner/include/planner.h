#ifndef PLANNER_H
#define PLANNER_H

// This seems to work for stage
#define ROBOT_SPEED   0.62

#define NORM_STEP     2
#define DIAG_STEP     2.8284271247461903
#define TIME_STEP     4

#define LETHAL_COST   100
#define OPEN_COST     0
#define UNKNOWN_COST  -1

#define SMOOTH_TOL    0.0000001
#define DATA_WEIGHT   0.5
#define SMOOTH_WEIGHT 0.5

// #include "node.h"

// #include "timemap_server/TimeLapseMap.h"
// #include "timemap_server/TimeLapseOccGrid.h"

// #include <ros/ros.h>
// #include <ros/console.h>

// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Point.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
// #include <nav_msgs/Path.h>

// #include <stdlib.h>
// #include <algorithm>
// #include <vector>
// #include <queue>
// #include <deque>

// class CellData{
// public:
// 	CellData(double d, double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
// 	distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
// 	{
// 	}
// 	double distance_;
// 	unsigned int index_;
// 	unsigned int x_, y_;
// 	unsigned int src_x_, src_y_;
// };

// inline bool operator<(const CellData &a, const CellData &b)
// {
// 	return a.distance_ > b.distance_;
// }

// namespace timeglobal_planner
// {
// 	class Planner {
// 	public:
// 		//path planning algorithm
// 		//returns false if path is trivial (start=end) or if no path is possible in the given time
// 		bool plan(const timemap_server::TimeLapseMap &map, nav_msgs::Path &path, Point start_pt, Point goal_pt);

// 	private:
		

// 		bool initialized_;

// 		ros::Subscriber map_sub_;
// 		ros::Subscriber goal_sub_;
// 		ros::Publisher path_pub_;

// 		timemap_server::TimeLapseMap map_;		

// 		Node start_;
// 		Node goal_;

// 		double resolution_;
// 		double time_res_;

// 		double origin_x_;
// 		double origin_y_;

// 		int size_x_;
// 		int size_y_;

// 		double cell_inflation_radius_;
// 		double cell_inflation_radius_m_;

// 		int end_time_;

// 		bool display_;
// 		int display_freq_;

// 		//The stuff below is mostly for debugging...
// 		ros::Publisher all_points_pub_;
// 		ros::Publisher processed_points_pub_;
// 		ros::Publisher best_points_pub_;
// 		ros::Publisher inflation_pub_;

// 		pcl::PointCloud<pcl::PointXYZ> processed_points_;
// 		pcl::PointCloud<pcl::PointXYZ> best_points_;
// 		pcl::PointCloud<pcl::PointXYZ> inflation_;

// 	protected:
// 		//setup callbacks and params
// 		Planner();

// 		//time_map callback...
// 		void timemap_callback(const timemap_server::TimeLapseMap &map);

// 		//goal callback...
// 		void goal_callback(const geometry_msgs::PoseStamped& goal);

		


// 		void inflate_map(timemap_server::TimeLapseMap &map);

// 		inline bool enqueue(unsigned char* grid, bool* seen, unsigned int index, std::priority_queue<CellData> &inflation_queue, unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y);

// 		double computeDistance(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1);

// 		unsigned char computeCost(double distance);

// 		//checks if occ value is acceptable
// 		inline bool valid(int val);

// 		//get the value for occupancy at a given location
// 		inline double get_occ(const timemap_server::TimeLapseMap &map, int x, int y, int t);

// 		//applies a path smoothing algorithm...
// 		void smooth_path(nav_msgs::Path &path);

// 		//fixes the header and publishes the path
// 		void publish_path(nav_msgs::Path &path);

// 		//Convert coordinates, taken with minor modifications from
// 		//the nav stack global planner
// 		void mapToWorld(int mx, int my, double& wx, double& wy);

// 		bool worldToMap(double wx, double wy, int& mx, int& my);

// 		void mapToWorld(int mx, int my, float& wx, float& wy);

// 		bool worldToMap(float wx, float wy, int& mx, int& my);
// 	};
// }

#endif