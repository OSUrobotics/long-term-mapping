#include "../include/potential_grid.h"

namespace timeglobal_planner
{
	PotentialGrid::PotentialGrid(){
		ros::NodeHandle private_nh("~");

		map_sub_ = private_nh.subscribe("/time_map", 1, &timeglobal_planner::PotentialGrid::timemap_callback, this);
		goal_sub_ = private_nh.subscribe("/nav_goal", 1, &timeglobal_planner::PotentialGrid::goal_callback, this);
		path_pub_ = private_nh.advertise<nav_msgs::Path>("/path", 1);

		private_nh.param<bool>("display", display_, false);

		if(display_){

			all_points_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("all_points", 1);
			processed_points_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("processed_points", 1);
			best_points_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("best_points", 1);

			all_points_.header.frame_id = "map";
			processed_points_.header.frame_id = "map";
			best_points_.header.frame_id = "map";

		}

		ros::spin();
	}

	void PotentialGrid::timemap_callback(const timemap_server::TimeLapseMap &map){
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

	void PotentialGrid::goal_callback(const geometry_msgs::PoseStamped& goal){			
		if(!initialized_){
			ROS_WARN("Path cannot be found: map not yet initialized");
			return;
		}
		nav_msgs::Path path;
		Point start_pt;
		Point goal_pt;

		//Static start point for now...
		start_pt.x = 2000;
		start_pt.y = 2000;
		start_pt.t = 0;

		if(!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_pt.x, goal_pt.y)){
			ROS_WARN("Goal point outside of known data, navigation will fail");
		}
		
		goal_pt.t  = 1000;

		ROS_DEBUG("Received goal at (%f, %f) for time %d\n", goal.pose.position.x, goal.pose.position.y, goal_pt.t);

		if(plan(map_, path, start_pt, goal_pt)){
			publish_path(path);
			ROS_DEBUG("Path published\n");
		}
	}

	bool PotentialGrid::plan(const timemap_server::TimeLapseMap &map, nav_msgs::Path &path, Point start_pt, Point goal_pt) {
		//this holds points that are neighbors of fully processed points
		std::vector< Point > pqueue;

		//this holds the points that have been processed
		std::vector< Layer > finished;

		Point cur;

		double full_time = ros::Time::now().toSec();

		ROS_DEBUG("Planning...\n");

		if(display_){
			all_points_.clear();
			processed_points_.clear();
		}

		start_     = start_pt;
		
		goal_      = goal_pt;

		pqueue.push_back(start_);
	
		while(!pqueue.empty()){
			//get the point closest to start
			cur = pqueue.front();
			std::pop_heap(pqueue.begin(), pqueue.end(), ComparePointsHeuristic(goal_));
			pqueue.pop_back();

			//we found the shortest path to the goal!
			if(cur == goal_){
				ROS_DEBUG("Time Required: %lf\n", ros::Time::now().toSec() - full_time);
				ROS_DEBUG("Retrieving path...");
				// return false;
				return get_path(path, cur, finished);
			}

			//adds the neighbors of the current point using the JPS algorithm
			add_neighbors(map, finished, pqueue, cur);

			if(display_){
				pcl::PointXYZ pt;
				
				mapToWorld(cur.x, cur.y, pt.x, pt.y);
				pt.z    = (0.01) * cur.t;;

				processed_points_.push_back(pt);
				
				processed_points_pub_.publish(processed_points_);
			}

			//mark point as completed
			add_finished(finished, cur);
		}

		ROS_WARN("Could not find path.");
		return false;
	}

	inline int PotentialGrid::get_endtime(const timemap_server::TimeLapseMap &map){
		//this comment makes sublime happy...
		return map.tmap.back().end;
	}

	inline void PotentialGrid::add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector< Layer > &finished, std::vector<Point> &pqueue, Point cur){
		//add forward position at next time
		add_neighbor(map, finished, pqueue, cur, 0, 1, NORM_STEP, 'f');

		//add backward location at next time
		add_neighbor(map, finished, pqueue, cur, 0, -1, NORM_STEP, 'b');

		//add left location at next time
		add_neighbor(map, finished, pqueue, cur, -1, 0, NORM_STEP, 'l');

		//add right location at next time
		add_neighbor(map, finished, pqueue, cur, 1, 0, NORM_STEP, 'r');
	}

	inline void PotentialGrid::add_neighbor(const timemap_server::TimeLapseMap &map, const std::vector< Layer > &finished, std::vector<Point> &pqueue, Point cur, int dx, int dy, int dt, char dir){
		Point new_point;

		new_point.x   = cur.x + dx;
		new_point.y   = cur.y + dy;
		new_point.t   = cur.t + dt;

		//if the corresponding cell is traversable
		if(valid(get_occ(map, new_point.x, new_point.y, new_point.t))){
			//if we haven't already processed the point
			if(!finished_point(finished, new_point)){
				add_point(pqueue, new_point);
			}
		}
	}

	inline void PotentialGrid::add_point(std::vector<Point> &pqueue, Point point){
		for(int i=0; i < pqueue.size(); i++){
			//update time and prev if new point is better than old
			if(pqueue[i] == point){
				if(pqueue[i].t > point.t){
					pqueue[i].t   = point.t;
				}
				return;
			} 
		}

		pqueue.push_back(point);
		std::push_heap(pqueue.begin(), pqueue.end(), ComparePointsHeuristic(goal_));

		if(display_){

			pcl::PointXYZ pt;
			
			mapToWorld(point.x, point.y, pt.x, pt.y);
			pt.z    = (0.01) * point.t;

			all_points_.push_back(pt);
			
			all_points_pub_.publish(all_points_);
		}
	}

	inline bool PotentialGrid::valid(int val){
		//this comment makes sublime happy...
		return (val != UNKNOWN_COST) && (val < LETHAL_COST);
	}

	inline double PotentialGrid::get_occ(const timemap_server::TimeLapseMap &map, int x, int y, int t){
		//find map that corresponds to the correct time
		int i=0;
		while(map.tmap[i].end < t){
			if(map.tmap.size() <= ++i){
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

	inline void PotentialGrid::add_finished(std::vector< Layer > &finished, Point cur){
		int index_x, index_y;

		//find the correct time and allocate memory

		//longest time seen yet
		if(cur.t + 1 > finished.size()){
			finished.resize(cur.t + 1);
		}

		//time that has not been used yet
		if(finished[cur.t].rows.size() == 0){
			Row new_row;

			new_row.y = cur.y;
			new_row.data.push_back(cur.t);
			finished[cur.t].x = cur.x;
			finished[cur.t].rows.push_back(new_row);

			return;
		}


		//find correct x and allocate memory

		//need to add row to front of deque
		if(cur.x < finished[cur.t].x){
			Row temp;
			for(int i = finished[cur.t].x - cur.x; i > 0; i--){
				finished[cur.t].rows.push_front(temp);
			}
			finished[cur.t].x = cur.x;
			index_x = 0;
		}

		//need to add row to back of deque
		else if(cur.x >= finished[cur.t].x + finished[cur.t].rows.size()){
			finished[cur.t].rows.resize(cur.x - finished[cur.t].x + 1);
			index_x = cur.x - finished[cur.t].x;
		}
		
		//row is already present...
		else{
			index_x = cur.x - finished[cur.t].x;
		}
		

		//find correct y and allocate memory

		//row that has not been used yet
		if(finished[cur.t].rows[index_x].data.size() == 0){
			finished[cur.t].rows[index_x].y = cur.y;
			finished[cur.t].rows[index_x].data.push_back(1);

			return;
		}

		//need to add point to front of deque
		if(cur.y < finished[cur.t].rows[index_x].y){
			for(int i = finished[cur.t].rows[index_x].y - cur.y; i > 0; i--){
				finished[cur.t].rows[index_x].data.push_front(0);
			}
			index_y = 0;
		}
		
		//need to add point to back of deque
		else if(cur.y >= finished[cur.t].rows[index_x].y + finished[cur.t].rows[index_x].data.size()){
			finished[cur.t].rows[index_x].data.resize(cur.y - finished[cur.t].rows[index_x].y + 1);
			index_y = cur.y - finished[cur.t].rows[index_x].y;
		}

		//need to add point to middle of deque
		else{
			index_y = cur.y - finished[cur.t].rows[index_x].y;
		}
		
		//add point
		finished[cur.t].rows[index_x].data[index_y] = 1;
	}

	inline bool PotentialGrid::finished_point(const std::vector< Layer > &finished, Point point){
		int index_x, index_y;

		//outside of time bounds
		if(finished.size() <= point.t){
			return false;
		}

		//within time bounds but no points have that specific time...
		if(finished[point.t].rows.size() == 0){
			return false;
		}

		//outside of x bounds
		if(point.x < finished[point.t].x){
			return false;
		}		

		if(point.x >= finished[point.t].x + finished[point.t].rows.size()){
			return false;
		}

		index_x = point.x - finished[point.t].x;

		//no points for given x value
		if(finished[point.t].rows[index_x].data.size() == 0){
			return false;
		}

		//outside of y bounds
		if(point.y < finished[point.t].rows[index_x].y){
			return false;
		}

		if(point.y >= finished[point.t].rows[index_x].y + finished[point.t].rows[index_x].data.size()){
			return false;
		}

		index_y = point.y - finished[point.t].rows[index_x].y;

		return finished[point.t].rows[index_x].data[index_y] == 1;
	}

	inline bool PotentialGrid::is_start(const Point point){
		//this comment makes sublime happy...
		return point.t == 0;
	}

	inline Point PotentialGrid::get_prev(const std::vector< Layer > &finished, Point cur){
		Point temp;
		Point min_pt;
		int min_time = cur.t;

		for(int dx = -1; dx <= 1; dx++){
			for(int dy = -1; dy <=1; dy++){
				temp.x = cur.x + dx;
				temp.y = cur.y + dy;

				// diagonal
				if(dx != 0 && dy != 0){
					continue;
					// temp.t = cur.t - DIAG_STEP;
				}
				// down
				else if(dx == 0 && dy == 0){
					continue;
					//temp.t = cur.t - TIME_STEP;
				}
				// cardinal direction
				else{
					temp.t = cur.t - NORM_STEP;
				}

				// check if neighbor exists
				if(finished_point(finished, temp)){

					// update min if necessary
					if(temp.t < min_time){
						min_time = temp.t;
		
						min_pt.x = temp.x;
						min_pt.y = temp.y;
						min_pt.t = temp.t;
					}
				}
			}
		}

		return min_pt;
	}

	bool PotentialGrid::get_path(nav_msgs::Path &path, Point goal, const std::vector< Layer > &finished){
		Point cur;

		cur = goal;

		//traceback through the path from goal to start
		while(!is_start(cur)){
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = ros::Time::now();
			pose.header.frame_id = "/map";

			mapToWorld(cur.x, cur.y, pose.pose.position.x, pose.pose.position.y);

			pose.pose.position.z = (0.01) * cur.t;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;

			path.poses.push_back(pose);

			cur = get_prev(finished, cur);
		}

		return !path.poses.empty();
	}

	void PotentialGrid::smooth_path(nav_msgs::Path &path){
		nav_msgs::Path new_path;
		new_path.poses.resize(path.poses.size());
		geometry_msgs::Point temp;

		for(int i = 0; i < path.poses.size(); i++){
			new_path.poses[i].pose.position.x = path.poses[i].pose.position.x;
			new_path.poses[i].pose.position.y = path.poses[i].pose.position.y;
			new_path.poses[i].pose.position.z = path.poses[i].pose.position.z;
		}

		// iterates until it converges to within a tolerance
		for(double change = 1; change >= SMOOTH_TOL;){
			change = 0;
			for(int i = 1; i < path.poses.size() - 1; i++){
				temp = new_path.poses[i].pose.position;

				// pulls back toward original point
				new_path.poses[i].pose.position.x += DATA_WEIGHT * (path.poses[i].pose.position.x - new_path.poses[i].pose.position.x);
				// pulls toward neighbors
				new_path.poses[i].pose.position.x += SMOOTH_WEIGHT * (new_path.poses[i - 1].pose.position.x + new_path.poses[i + 1].pose.position.x - 2.0 * new_path.poses[i].pose.position.x);
				// records difference
				change += abs(temp.x - new_path.poses[i].pose.position.x);

				new_path.poses[i].pose.position.y += DATA_WEIGHT * (path.poses[i].pose.position.y - new_path.poses[i].pose.position.y);
				new_path.poses[i].pose.position.y += SMOOTH_WEIGHT * (new_path.poses[i - 1].pose.position.y + new_path.poses[i + 1].pose.position.y - 2.0 * new_path.poses[i].pose.position.y);
				change += abs(temp.y - new_path.poses[i].pose.position.y);

				new_path.poses[i].pose.position.z += DATA_WEIGHT * (path.poses[i].pose.position.z - new_path.poses[i].pose.position.z);
				new_path.poses[i].pose.position.z += SMOOTH_WEIGHT * (new_path.poses[i - 1].pose.position.z + new_path.poses[i + 1].pose.position.z - 2.0 * new_path.poses[i].pose.position.z);
				change += abs(temp.z - new_path.poses[i].pose.position.z);
			}
		}

		for(int i = 0; i < path.poses.size(); i++){
			path.poses[i].pose.position.x = new_path.poses[i].pose.position.x;
			path.poses[i].pose.position.y = new_path.poses[i].pose.position.y;
			path.poses[i].pose.position.z = new_path.poses[i].pose.position.z;
		}
	}
	
	void PotentialGrid::publish_path(nav_msgs::Path &path){
		path.header.frame_id = path.poses[0].header.frame_id;
		path.header.stamp    = path.poses[0].header.stamp;

		path_pub_.publish(path);
	}

	//these functions are from the nav stack...
	void PotentialGrid::mapToWorld(int mx, int my, double& wx, double& wy){
		wx = origin_x_ + (mx + 0.5) * resolution_;
		wy = origin_y_ + (my + 0.5) * resolution_;
	}

	bool PotentialGrid::worldToMap(double wx, double wy, int& mx, int& my){
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

	void PotentialGrid::mapToWorld(int mx, int my, float& wx, float& wy){
		wx = origin_x_ + (mx + 0.5) * resolution_;
		wy = origin_y_ + (my + 0.5) * resolution_;
	}

	bool PotentialGrid::worldToMap(float wx, float wy, int& mx, int& my){
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