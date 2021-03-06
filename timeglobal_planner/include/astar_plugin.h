
#ifndef ASTAR_H
#define ASTAR_H

#include "node.h"
#include "planner.h"

#include "timemap_server/TimeLapseMap.h"
#include "timemap_server/TimeLapseOccGrid.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h> 
#include <nav_core/base_global_planner.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>

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


class CompareNodesHeuristic {
public:
	CompareNodesHeuristic(Node new_goal) : goal(new_goal){}
	// Diagonal distance
	inline int heuristic(const Node& cur){
		int dx = abs(cur.pt.x - goal.pt.x);
		int dy = abs(cur.pt.y - goal.pt.y);

	 // Manhattan distance with the distance saved using diagonals subtracted off
		return (NORM_STEP * (dx + dy) + (DIAG_STEP - 2 * NORM_STEP) * std::min(dx, dy));
	}

	// Manhattan Distance
	// inline int heuristic(const Node& cur){
	// 	int dx = abs(cur.pt.x - goal.pt.x);
	// 	int dy = abs(cur.pt.y - goal.pt.y);

	// 	return 3 * (NORM_STEP * (dx + dy));
	// }

	bool const operator()(const Node &lhs, const Node &rhs) {
		return (lhs.pt.t + 3 * heuristic(lhs) > rhs.pt.t + 3 * heuristic(rhs));
	}

private:
	Node goal;

};

namespace timeglobal_planner
{
	class AStarPlanner : public nav_core::BaseGlobalPlanner{
	public:
		//setup callbacks and params
		AStarPlanner();

		AStarPlanner(std::string name);

		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan); 

		//time_map callback...
		void timemap_callback(const timemap_server::TimeLapseMap &map);

	private:
		//gets the last time described by the current map set
		inline int get_endtime(const timemap_server::TimeLapseMap &map);

		//will update dists as neighbors are added. neighbors that are added will have prev set to cur
		//note: neighbors are in time levels below cur based on the values of their respective movement
		inline void add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector< std::deque< std::deque< Node > > > &finished, std::vector<Node> &pqueue, Node cur);

		//adds neighbors according to JPS algorithm
		inline void add_neighbor(const timemap_server::TimeLapseMap &map, const std::vector< std::deque< std::deque< Node > > > &finished, std::vector<Node> &pqueue, Node cur, int dx, int dy, int dt);

		//adds node to neighbors. if already present, updates dist
		inline void add_node(std::vector<Node> &pqueue, Node node);

		//checks if occ value is acceptable
		inline bool valid(int val);

		//get the value for occupancy at a given location
		inline double get_occ(const timemap_server::TimeLapseMap &map, int x, int y, int t);

		//add the node to the list of finished nodes
		inline void add_finished(std::vector< std::deque< std::deque< Node > > > &finished, Node cur);

		//if the node has been fully processed
		inline bool finished_node(const std::vector< std::deque< std::deque< Node > > > &finished, Node node);

		//if it is the start node
		inline bool is_start(const Node node);

		//get the parent node
		inline Node get_prev(const std::vector< std::deque< std::deque< Node > > > &finished, Node cur);

		//traces back the plan and returns a ROS plan
		bool get_plan(std::vector<geometry_msgs::PoseStamped>& plan, Node goal, const std::vector< std::deque< std::deque< Node > > > &finished);

		//applies a plan smoothing algorithm...
		void smooth_plan(std::vector<geometry_msgs::PoseStamped>& plan);

		//Convert coordinates, taken with minor modifications from
		//the nav stack global planner
		void mapToWorld(int mx, int my, double& wx, double& wy);

		bool worldToMap(double wx, double wy, int& mx, int& my);

		void mapToWorld(int mx, int my, float& wx, float& wy);

		bool worldToMap(float wx, float wy, int& mx, int& my);

		bool initialized_;
		bool has_map_;

		ros::Subscriber map_sub_;

		timemap_server::TimeLapseMap map_;		

		Node start_;
		Node goal_;

		double resolution_;

		double origin_x_;
		double origin_y_;

		int size_x_;
		int size_y_;

		int end_time_;

		bool display_;

		//The stuff below is mostly for debugging...
		ros::Publisher all_points_pub_;
		ros::Publisher processed_points_pub_;
		ros::Publisher best_points_pub_;

		pcl::PointCloud<pcl::PointXYZ> all_points_;
		pcl::PointCloud<pcl::PointXYZ> processed_points_;
		pcl::PointCloud<pcl::PointXYZ> best_points_;

		double time_1, time_2, time_3, time_4, time_5, time_6, time_7;
		int iter_1, iter_2, iter_3, iter_4, iter_5, iter_6, iter_7;

	};
}

#endif