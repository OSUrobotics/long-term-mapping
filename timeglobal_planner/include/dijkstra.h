
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "node.h"

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

#include <algorithm>
#include <vector>
#include <queue>

// Steps in time required

//TODO: params!!!

// #define DISPLAY

#define NORM_STEP    2
#define DIAG_STEP    3
#define TIME_STEP    1

#define LETHAL_COST  100
#define OPEN_COST    0
#define UNKNOWN_COST -1


// #pragma message "in header!"

namespace timeglobal_planner
{
	class CompareNodes {
	public:
		bool const operator()(const Node &lhs, const Node &rhs) {return (lhs.pt.t > rhs.pt.t);}
	};

	class Dijkstra {
	public:
		Dijkstra();
		//time_map callback...
		void timemap_callback(const timemap_server::TimeLapseMap &map);

		//goal callback...
		void goal_callback(const geometry_msgs::PoseStamped& goal);

		//path planning algorithm... hopefully
		//returns false if path is trivial (start=end) or if no path is possible in timeframe
		bool plan(const timemap_server::TimeLapseMap &map, nav_msgs::Path &path, Point start_pt, Point goal_pt);

	private:
		//will update dists as neighbors are added. neighbors that are added will have prev set to cur
		//note: neighbors are in time levels below cur based on the values of their respective movement
		inline void add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector<Node> &finished, std::vector<Node> &pqueue, Node cur, int index);

		//as implied..
		inline void add_neighbor(const timemap_server::TimeLapseMap &map, const std::vector<Node> &finished, std::vector<Node> &pqueue, Node cur, int index, int dx, int dy, int dt);

		//adds node to neighbors. if already present, updates dist
		inline void add_node(std::vector<Node> &pqueue, Node node);

		//checks if occ value is acceptable
		inline bool valid(int val){return (val != UNKNOWN_COST) && (val < LETHAL_COST);}

		//get the value for occupancy at a given location
		inline double get_occ(const timemap_server::TimeLapseMap &map, int x, int y, int t);

		//traces back the path and returns a list of waypoints
		bool get_path(nav_msgs::Path &path, Node goal, const std::vector<Node> &finished);

		void publish_path(nav_msgs::Path &path);

		//checks if node has been fully processed
		inline bool finished_node(const std::vector<Node> &finished, Node node);

		inline int get_endtime(const timemap_server::TimeLapseMap &map);

		void mapToWorld(int mx, int my, double& wx, double& wy);

		bool worldToMap(double wx, double wy, int& mx, int& my);

		void mapToWorld(int mx, int my, float& wx, float& wy);

		bool worldToMap(float wx, float wy, int& mx, int& my);

		bool initialized_;

		ros::Subscriber map_sub_;
		ros::Subscriber goal_sub_;
		ros::Publisher path_pub_;

		timemap_server::TimeLapseMap map_;		

		double resolution_;

		double origin_x_;
		double origin_y_;

		int size_x_;
		int size_y_;

		int end_time_;

		#ifdef DISPLAY
		ros::Publisher test_pub_;
		ros::Publisher test_pub2_;

		pcl::PointCloud<pcl::PointXYZ> pt_cloud_;
		pcl::PointCloud<pcl::PointXYZ> pt_cloud2_;
		#endif

		double time_1, time_2, time_3, time_4, time_5, time_6, time_7;
		int iter_1, iter_2, iter_3, iter_4, iter_5, iter_6, iter_7;

	};
}

#endif