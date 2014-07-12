#include "../include/astar.h"

namespace timeglobal_planner
{
	AStar::AStar(){
		ros::NodeHandle private_nh("~");

		map_sub_ = private_nh.subscribe("/time_map", 1, &timeglobal_planner::AStar::timemap_callback, this);
		goal_sub_ = private_nh.subscribe("/nav_goal", 1, &timeglobal_planner::AStar::goal_callback, this);
		path_pub_ = private_nh.advertise<nav_msgs::Path>("/path", 1);

		#ifdef DISPLAY

			test_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_test", 1);
			test_pub2_ = private_nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_test2", 1);

			pt_cloud_.header.frame_id = "map";
			pt_cloud2_.header.frame_id = "map";

		#endif

		ros::spin();
	}

	void AStar::timemap_callback(const timemap_server::TimeLapseMap &map){
		if(map.tmap.size() == 0){
			ROS_ERROR("Map cannot be empty\n");
			return;
		}

		map_         = map;

		resolution_  = map.tmap[0].map.info.resolution;

		origin_x_    = map.tmap[0].map.info.origin.position.x;
		origin_y_    = map.tmap[0].map.info.origin.position.y;

		size_x_      = map.tmap[0].map.info.width;
		size_y_      = map.tmap[0].map.info.height;

		end_time_    = get_endtime(map);

		
		ROS_DEBUG("Received %lu maps of size %d X %d and duration %d\n", map.tmap.size(), size_x_, size_y_, end_time_);

		initialized_ = true;
	}

	void AStar::goal_callback(const geometry_msgs::PoseStamped& goal){			
		if(!initialized_){
			ROS_WARN("Path cannot be found: map not yet initialized");
			return;
		}
		nav_msgs::Path path;
		Point start_pt;
		Point goal_pt;

		start_pt.x = 2000;
		start_pt.y = 2000;
		start_pt.t = 0;

		if(!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_pt.x, goal_pt.y)){
			ROS_WARN("Goal point outside of known data, navigation will fail");
		}
		;
		goal_pt.t  = 1000;

		
		ROS_DEBUG("Received goal at (%f, %f) for time %d\n", goal.pose.position.x, goal.pose.position.y, goal_pt.t);

		if(plan(map_, path, start_pt, goal_pt)){
			publish_path(path);
			ROS_DEBUG("Path published")
		}

	bool AStar::plan(const timemap_server::TimeLapseMap &map, nav_msgs::Path &path, Point start_pt, Point goal_pt) {
		
		ROS_DEBUG("Planning...\n");

		#ifdef DISPLAY

			pt_cloud_.clear();
			pt_cloud2_.clear();
		
		#endif

		// Used to time code...
		time_1 = 0;
		iter_1 = 0;

		time_2 = 0;
		iter_2 = 0;

		time_3 = 0;
		iter_3 = 0;

		time_4 = 0;
		iter_4 = 0;

		// double start_time =ros::Time::now().toSec();

		std::vector<Node> pqueue;
		std::vector<Node> finished;

		Node cur;

		start_.pt = start_pt;
		start_.prev = -1;

		goal_.pt = goal_pt;
		goal_.prev = -1;

		pqueue.push_back(start_);

		// double end_lsd = ros::Time::now().toSec();
		// ROS_DEBUG("1: %lf", end_lsd - start_time);

	
		int i = 0;
		while(!pqueue.empty()){
			ROS_DEBUG("time: %d", cur.pt.t);

			// ROS_DEBUG("len heap: %lu", pqueue.size());
			// ROS_DEBUG("len finished: %lu", pqueue.size());

			//get the node closest to start
			cur = pqueue.front();
			std::pop_heap(pqueue.begin(), pqueue.end(), CompareNodesHeuristic(goal_));
			pqueue.pop_back();

			//we found the shortest path to the goal!
			if(cur == goal_){
				ROS_DEBUG("Retrieving path...");
				return get_path(path, cur, finished);
			}

			// start_time =ros::Time::now().toSec();

			//add_neighbors will update the dists if an already added node is added twice
			//and the new node has an improved dist
			add_neighbors(map, finished, pqueue, cur, i);

			// end_lsd = ros::Time::now().toSec();
			// ROS_DEBUG("4: %lf", end_lsd - start_time);

			#ifdef DISPLAY

				pcl::PointXYZ pt;
				
				mapToWorld(cur.pt.x, cur.pt.y, pt.x, pt.y);
				pt.z    = (0.01) * cur.pt.t;;

				pt_cloud2_.push_back(pt);
				
				test_pub2_.publish(pt_cloud2_);

			#endif

			//mark node as completed
			finished.push_back(cur);

			i++;
		}

		ROS_WARN("Could not find path.");
		return false;
	}

	inline void AStar::add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector<Node> &finished, std::vector<Node> &pqueue, Node cur, int index){
		//add current location at next time
		add_neighbor(map, finished, pqueue, cur, index, 0, 0, TIME_STEP);	
		
		//add up location at next time
		add_neighbor(map, finished, pqueue, cur, index, 0, 1, NORM_STEP);

		//add down location at next time
		add_neighbor(map, finished, pqueue, cur, index, 0, -1, NORM_STEP);

		//add left location at next time
		add_neighbor(map, finished, pqueue, cur, index, -1, 0, NORM_STEP);

		//add right location at next time
		add_neighbor(map, finished, pqueue, cur, index, 1, 0, NORM_STEP);

		//add up left location at next time
		add_neighbor(map, finished, pqueue, cur, index, -1, 1, DIAG_STEP);

		//add up right location at next time
		add_neighbor(map, finished, pqueue, cur, index, 1, 1, DIAG_STEP);

		//add down left location at next time
		add_neighbor(map, finished, pqueue, cur, index, -1, -1, DIAG_STEP);

		//add current location at next time
		add_neighbor(map, finished, pqueue, cur, index, 1, -1, DIAG_STEP);
	}

	inline void AStar::add_neighbor(const timemap_server::TimeLapseMap &map, const std::vector<Node> &finished, std::vector<Node> &pqueue, Node cur, int index, int dx, int dy, int dt){
		// double start_time =ros::Time::now().toSec();
		Node new_node;

		new_node.pt.x = cur.pt.x + dx;
		new_node.pt.y = cur.pt.y + dy;
		new_node.pt.t = cur.pt.t + dt;
		new_node.prev = index;

		// double end_lsd = ros::Time::now().toSec();
		// time_1 += end_lsd - start_time;
		// ROS_DEBUG("add_neighbor - 1: %lf", time_1 / ++iter_1);

		// start_time =ros::Time::now().toSec();

		if(valid(get_occ(map, new_node.pt.x, new_node.pt.y, new_node.pt.t))){
			// end_lsd = ros::Time::now().toSec();
			// time_2 += end_lsd - start_time;
			// ROS_DEBUG("add_neighbor - 2: %lf", time_2 / ++iter_2);
			// start_time =ros::Time::now().toSec();

			
			if(!finished_node(finished, new_node)){

				// end_lsd = ros::Time::now().toSec();
				// time_3 += end_lsd - start_time;
				// ROS_DEBUG("add_neighbor - 3: %lf", time_3 / ++iter_3);
				// start_time =ros::Time::now().toSec();
				
				add_node(pqueue, new_node);
				// end_lsd = ros::Time::now().toSec();
				// time_4 += end_lsd - start_time;
		// ROS_DEBUG("add_neighbor - 4: %lf", time_4 / ++iter_4);
				
			}
		}
	}

	inline void AStar::add_node(std::vector<Node> &pqueue, Node node){
		for(int i=0; i < pqueue.size(); i++){
			//update dist and prev if new node is better than old
			if(pqueue[i] == node){
				if(pqueue[i].pt.t > node.pt.t){
					pqueue[i].prev = node.prev;
					pqueue[i].pt.t = node.pt.t;
				}
				return;
			} 
		}

		#ifdef DISPLAY

			pcl::PointXYZ pt;
			
			mapToWorld(node.pt.x, node.pt.y, pt.x, pt.y);
			pt.z    = (0.01) * node.pt.t;

			pt_cloud_.push_back(pt);
			
			test_pub_.publish(pt_cloud_);

		#endif

		pqueue.push_back(node);
		std::push_heap(pqueue.begin(), pqueue.end(), CompareNodesHeuristic(goal_));
	}

	bool AStar::get_path(nav_msgs::Path &path, Node goal, const std::vector<Node> &finished){
		int cur;

		cur = goal.prev;

		//traceback through the path from goal to start
		while(cur != -1){
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = ros::Time::now();
			pose.header.frame_id = "/map";

			mapToWorld(finished[cur].pt.x, finished[cur].pt.y, pose.pose.position.x, pose.pose.position.y);

			pose.pose.position.z = (0.01) * finished[cur].pt.t;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;

			path.poses.push_back(pose);

			cur = finished[cur].prev;
		}

		return !path.poses.empty();
	}

	inline int AStar::get_endtime(const timemap_server::TimeLapseMap &map){
		//this comment makes sublime happy...
		return map.tmap.back().end;
	}

	void AStar::publish_path(nav_msgs::Path &path){
		path.header.frame_id = path.poses[0].header.frame_id;
		path.header.stamp    = path.poses[0].header.stamp;

		path_pub_.publish(path);
	}

	inline bool AStar::finished_node(const std::vector<Node> &finished, Node node){
		//navigate list backwards because most recent nodes are at back
		for(int i=finished.size() - 1; i >= 0; i--){
			if(finished[i] == node && finished[i].pt.t == node.pt.t){
				return true;
			}
		}
		return false;
	}

	inline double AStar::get_occ(const timemap_server::TimeLapseMap &map, int x, int y, int t){
		int ny;

		//find map that corresponds to the correct time
		int i=0;
		while(map.tmap[i].end < t){
			if(map.tmap.size() <= ++i){
				//TODO: make it possible to navigate with the base map when end time is exceeded
				//we do not have enough data to navigate to destination..
				ROS_WARN("Insufficient map data. End time has been exceeded.");
				return -1;
			}
		}

		//find cell in map that corresponds
		if(size_y_ * y + x >= 0 && size_y_ * y + x < map.tmap[i].map.data.size()){
			return map.tmap[i].map.data[size_y_ * y + x];
		}
		else{
			ROS_WARN("Attempted to access data outside of map bounds.");
			return -1;
		}
	}

	void AStar::mapToWorld(int mx, int my, double& wx, double& wy){
		wx = origin_x_ + (mx + 0.5) * resolution_;
		wy = origin_y_ + (my + 0.5) * resolution_;
	}

	bool AStar::worldToMap(double wx, double wy, int& mx, int& my){
		if(wx < origin_x_ || wy < origin_y_){
			return false;
		}

		mx = (int)((wx - origin_x_) / resolution_);
		my = (int)((wy - origin_y_) / resolution_);

		if(mx < size_x_ && my < size_y_){
			return true;
		}
		return false;
	}

	void AStar::mapToWorld(int mx, int my, float& wx, float& wy){
		wx = origin_x_ + (mx + 0.5) * resolution_;
		wy = origin_y_ + (my + 0.5) * resolution_;
	}

	bool AStar::worldToMap(float wx, float wy, int& mx, int& my){
		if(wx < origin_x_ || wy < origin_y_){
			return false;
		}

		mx = (int)((wx - origin_x_) / resolution_);
		my = (int)((wy - origin_y_) / resolution_);

		if(mx < size_x_ && my < size_y_){
			return true;
		}
		return false;
	}
}