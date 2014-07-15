
#ifndef POTENTIAL_GRID_H
#define POTENTIAL_GRID_H

#include "node.h"
#include "planner.h"

#include "timemap_server/TimeLapseMap.h"
#include "timemap_server/TimeLapseOccGrid.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Path.h>

#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <deque>

class ComparePointsHeuristic {
public:
	ComparePointsHeuristic(Point new_goal){goal = new_goal;}

	inline int heuristic(const Point& cur){
		int dx = abs(cur.x - goal.x);
		int dy = abs(cur.y - goal.y);

		return NORM_STEP * (dx + dy) + (DIAG_STEP - 2 * NORM_STEP) * std::min(dx, dy);
	}

	bool const operator()(const Point &lhs, const Point &rhs) {
		return (lhs.t + heuristic(lhs) > rhs.t + heuristic(rhs));
	}

private:
	Point goal;

};

namespace timeglobal_planner
{
	class PotentialGrid {
	public:
		//setup callbacks and params
		PotentialGrid();

		//time_map callback...
		void timemap_callback(const timemap_server::TimeLapseMap &map);

		//goal callback...
		void goal_callback(const geometry_msgs::PoseStamped& goal);

		//path planning algorithm
		//returns false if path is trivial (start=end) or if no path is possible in the given time
		bool plan(const timemap_server::TimeLapseMap &map, nav_msgs::Path &path, Point start_pt, Point goal_pt);

	private:
		//gets the last time described by the current map set
		inline int get_endtime(const timemap_server::TimeLapseMap &map);

		//will update dists as neighbors are added. neighbors that are added will have prev set to cur
		//note: neighbors are in time levels below cur based on the values of their respective movement
		inline void add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector< Layer > &finished, std::vector<Point> &pqueue, Point cur);

		//adds neighbors according to JPS algorithm
		inline void add_neighbor(const timemap_server::TimeLapseMap &map, const std::vector< Layer > &finished, std::vector<Point> &pqueue, Point cur, int dx, int dy, int dt, char dir);

		//adds point to neighbors. if already present, updates dist
		inline void add_point(std::vector<Point> &pqueue, Point point);

		//checks if occ value is acceptable
		inline bool valid(int val);

		//get the value for occupancy at a given location
		inline double get_occ(const timemap_server::TimeLapseMap &map, int x, int y, int t);

		//add the point to the list of finished points
		inline void add_finished(std::vector< Layer > &finished, Point cur);

		//if the point has been fully processed
		inline bool finished_point(const std::vector< Layer > &finished, Point point);

		//if it is the start point
		inline bool is_start(const Point point);

		//get the parent point
		inline Point get_prev(const std::vector< Layer > &finished, Point cur);

		//traces back the path and returns a ROS path
		bool get_path(nav_msgs::Path &path, Point goal, const std::vector< Layer > &finished);

		//fixes the header and publishes the path
		void publish_path(nav_msgs::Path &path);

		//Convert coordinates, taken with minor modifications from
		//the nav stack global planner
		void mapToWorld(int mx, int my, double& wx, double& wy);

		bool worldToMap(double wx, double wy, int& mx, int& my);

		void mapToWorld(int mx, int my, float& wx, float& wy);

		bool worldToMap(float wx, float wy, int& mx, int& my);

		bool initialized_;

		ros::Subscriber map_sub_;
		ros::Subscriber goal_sub_;
		ros::Publisher path_pub_;

		timemap_server::TimeLapseMap map_;		

		Point start_;
		Point goal_;

		double resolution_;

		double origin_x_;
		double origin_y_;

		int size_x_;
		int size_y_;

		int end_time_;

		bool display_;

		//The stuff below is mostly for debugging...

		ros::Publisher test_pub_;
		ros::Publisher test_pub2_;

		pcl::PointCloud<pcl::PointXYZ> pt_cloud_;
		pcl::PointCloud<pcl::PointXYZ> pt_cloud2_;

		double time_1, time_2, time_3, time_4, time_5, time_6, time_7;
		int iter_1, iter_2, iter_3, iter_4, iter_5, iter_6, iter_7;

	};
}

#endif