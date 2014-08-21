#include "../include/astar.h"

namespace timeglobal_planner
{
	AStar::AStar(){
		ROS_DEBUG("Initializing planner...");
		ros::NodeHandle private_nh("~");

		map_sub_ = private_nh.subscribe("/time_map", 1, &timeglobal_planner::AStar::timemap_callback, this);
		goal_sub_ = private_nh.subscribe("/planner/nav_goal", 1, &timeglobal_planner::AStar::goal_callback, this);
		path_pub_ = private_nh.advertise<nav_msgs::Path>("/planner/path", 1);
		inflation_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("/planner/inflation", 1);
		inflation_.header.frame_id = "map";

		private_nh.param<bool>("display", display_, false);
		private_nh.param<int>("display_freq", display_freq_, 10);
		// cell inflation radius is in meters
		private_nh.param<double>("cell_inflation_radius", cell_inflation_radius_m_, 0.5);

		if(display_){
			ROS_DEBUG("display: true");
			processed_points_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("processed_points", 1);
			best_points_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("best_points", 1);

			processed_points_.header.frame_id = "map";
			best_points_.header.frame_id = "map";
			ROS_DEBUG("display_freq: %d", display_freq_);

		}
		else{
			ROS_DEBUG("display: false");
		}

		ROS_DEBUG("cell_inflation_radius: %f", cell_inflation_radius_m_);

		ros::spin();
	}

	void AStar::timemap_callback(const timemap_server::TimeLapseMap &map){
		if(map.tmap.size() == 0){
			ROS_ERROR("Map cannot be empty\n");
			return;
		}

		map_         = map;

		resolution_  = map.tmap[0].map.info.resolution;
		time_res_    = resolution_ / (ROBOT_SPEED * NORM_STEP);

		origin_x_    = map.tmap[0].map.info.origin.position.x;
		origin_y_    = map.tmap[0].map.info.origin.position.y;

		size_x_      = map.tmap[0].map.info.width;
		size_y_      = map.tmap[0].map.info.height;

		start_time_  = map.tmap[0].begin.toSec();
		end_time_    = map.tmap.back().end.toSec();

		cell_inflation_radius_ = cell_inflation_radius_m_ / resolution_;

		inflate_map(map_);
		
		ROS_DEBUG("Received %lu maps of size %d X %d and duration %f\n", map.tmap.size(), size_x_, size_y_, end_time_ - start_time_);

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

		//Static start point for now...
		start_pt.x = 2000;
		start_pt.y = 2000;
		start_pt.t = 0;

		if(!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_pt.x, goal_pt.y)){
			ROS_WARN("Goal point outside of known data, navigation will fail");
		}

		ROS_DEBUG("Received goal at (%f, %f)", goal.pose.position.x, goal.pose.position.y);

		if(plan(map_, path, start_pt, goal_pt)){
			publish_path(path);
			ROS_DEBUG("Path published\n");
		}
	}

	bool AStar::plan(const timemap_server::TimeLapseMap &map, nav_msgs::Path &path, Point start_pt, Point goal_pt) {
		//this holds nodes that are neighbors of fully processed nodes
		std::vector< Node > pqueue;

		//this holds the nodes that have been processed
		std::vector< std::deque< std::deque< Node > > > finished;

		Node cur;

		double full_time = ros::Time::now().toSec();

		ROS_DEBUG("Planning...\n");

		if(display_){
			processed_points_.clear();
		}

		inflation_pub_.publish(inflation_);

		start_.pt     = start_pt;
		start_.prev.t = -1;
		start_.dir    = 'u';
		
		goal_.pt      = goal_pt;

		pqueue.push_back(start_);
	
		int j = 0;
		while(!pqueue.empty() && ros::ok()){
			//get the node closest to start
			cur = pqueue.front();
			std::pop_heap(pqueue.begin(), pqueue.end(), CompareNodesHeuristic(goal_));
			pqueue.pop_back();

			//we found the shortest path to the goal!
			if(cur == goal_){
				if(display_){
					processed_points_pub_.publish(processed_points_);
				}

				ROS_DEBUG("Time Required: %lf\n", ros::Time::now().toSec() - full_time);
				ROS_DEBUG("Retrieving path...");
				return get_path(path, cur, finished);
			}

			//adds the neighbors of the current node using the JPS algorithm
			add_neighbors(map, finished, pqueue, cur);

			if(display_){
				pcl::PointXYZ pt;
				
				mapToWorld(cur.pt.x, cur.pt.y, pt.x, pt.y);
				pt.z = (0.01) * cur.pt.t;

				processed_points_.push_back(pt);

				if(!(++j % display_freq_)){
					processed_points_pub_.publish(processed_points_);
					j = 0;
				}

				for(int i=std::min(5, int(pqueue.size() - 1)); i >= 0; i--){
					mapToWorld(pqueue[i].pt.x, pqueue[i].pt.y, pt.x, pt.y);
					pt.z = (0.01) * pqueue[i].pt.t;

					best_points_.push_back(pt);
				}

				best_points_pub_.publish(best_points_);

				best_points_.clear();

			}			

			//mark node as completed
			add_finished(finished, cur);
		}

		ROS_WARN("Could not find path.");
		return false;
	}

	inline double AStar::get_endtime(const timemap_server::TimeLapseMap &map){
		//this comment makes sublime happy...
		return map.tmap.back().end.toSec();
	}

	// times:
	// 2.097859 - (-4.985855, 2.975908)
	// 1.317784 - (-4.016930, 3.020894)
	// 3.556497 - ( 0.983357, 3.031751)
	inline void AStar::add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector< std::deque< std::deque< Node > > > &finished, std::vector<Node> &pqueue, Node cur){
		//add forward position at next time
		add_neighbor(map, finished, pqueue, cur, 0, 1, NORM_STEP, 'f');

		//add backward location at next time
		add_neighbor(map, finished, pqueue, cur, 0, -1, NORM_STEP, 'b');

		//add left location at next time
		add_neighbor(map, finished, pqueue, cur, -1, 0, NORM_STEP, 'l');

		//add right location at next time
		add_neighbor(map, finished, pqueue, cur, 1, 0, NORM_STEP, 'r');
	}

	// times:
	// 3.098328 - (-4.986468, 2.981237)
	// 1.960431 - (-3.983218, 3.016832)
	// inline void AStar::add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector< std::deque< std::deque< Node > > > &finished, std::vector<Node> &pqueue, Node cur){
	// 	//add current location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 0, 0, TIME_STEP, 'u');	

	// 	//add forward position at next time
	// 	add_neighbor(map, finished, pqueue, cur, 0, 1, NORM_STEP, 'f');

	// 	//add backward location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 0, -1, NORM_STEP, 'b');

	// 	//add left location at next time
	// 	add_neighbor(map, finished, pqueue, cur, -1, 0, NORM_STEP, 'l');

	// 	//add right location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 1, 0, NORM_STEP, 'r');
	// }


	// times:
	// 8.249100 - (-4.986468, 3.030174)
	// 4.697925 - (-4.007739, 3.013862)

	// inline void AStar::add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector< std::deque< std::deque< Node > > > &finished, std::vector<Node> &pqueue, Node cur){
	// 	//add current location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 0, 0, TIME_STEP, 'u');	

	// 	//add forward position at next time
	// 	add_neighbor(map, finished, pqueue, cur, 0, 1, NORM_STEP, 'f');

	// 	//add backward location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 0, -1, NORM_STEP, 'b');

	// 	//add left location at next time
	// 	add_neighbor(map, finished, pqueue, cur, -1, 0, NORM_STEP, 'l');

	// 	//add right location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 1, 0, NORM_STEP, 'r');

	// 	//add forward left location at next time
	// 	add_neighbor(map, finished, pqueue, cur, -1, 1, DIAG_STEP, 'q');

	// 	//add forward right location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 1, 1, DIAG_STEP, 'e');

	// 	//add back left location at next time
	// 	add_neighbor(map, finished, pqueue, cur, -1, -1, DIAG_STEP, 'z');

	// 	//add back right location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 1, -1, DIAG_STEP, 'c');
	// }


	// times:
	// 3.621211 - (-5.005533, 3.015647)
	// 1.436171 - (-4.018334, 3.044444)
	// 8.449397 - ( 0.962736, 3.008032)

	// inline void AStar::add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector< std::deque< std::deque< Node > > > &finished, std::vector<Node> &pqueue, Node cur){
	// 	//This implements a Jump Point Search neighbor selection. This ignores nodes that
	// 	//can be traveled to more quickly through another node. The algorithm takes into
	// 	//account where the parent node is to determine where the node is going. Using this
	// 	//it culls the nodes that are unnecessary. When traveling in cardinal directions,
	// 	//only consider the node directly in front. When traveling diagonally, only consider
	// 	//the node in front and the nodes bordering it. Obstacles can create blocked nodes 
	// 	//which must also be considered.

	// 	//directions:
	// 	//  q f e
	// 	//  l u r
	// 	//  z b c

	// 	//add current location at next time
	// 	add_neighbor(map, finished, pqueue, cur, 0, 0, TIME_STEP, 'u');	

	// 	//add forward position at next time
	// 	if(cur.dir == 'u' || cur.dir == 'f' || cur.dir == 'q' || cur.dir == 'e'){
	// 		add_neighbor(map, finished, pqueue, cur, 0, 1, NORM_STEP, 'f');
	// 	}

	// 	//add backward location at next time
	// 	if(cur.dir == 'u' || cur.dir == 'b' || cur.dir == 'z' || cur.dir == 'c'){
	// 		add_neighbor(map, finished, pqueue, cur, 0, -1, NORM_STEP, 'b');
	// 	}

	// 	//add left location at next time
	// 	if(cur.dir == 'u' || cur.dir == 'l' || cur.dir == 'q' || cur.dir == 'z'){
	// 		add_neighbor(map, finished, pqueue, cur, -1, 0, NORM_STEP, 'l');
	// 	}

	// 	//add right location at next time
	// 	if(cur.dir == 'u' || cur.dir == 'r' || cur.dir == 'e' || cur.dir == 'c'){
	// 		add_neighbor(map, finished, pqueue, cur, 1, 0, NORM_STEP, 'r');
	// 	}

	// 	//add forward left location at next time
	// 	if(cur.dir == 'u' || cur.dir == 'q'
	// 		//if certain neighbors are blocked, maybe include
	// 		|| !valid(get_occ(map, cur.pt.x - 1, cur.pt.y, cur.pt.t)) && cur.dir == 'f'
	// 		|| !valid(get_occ(map, cur.pt.x, cur.pt.y + 1, cur.pt.t)) && cur.dir == 'l'){

	// 		add_neighbor(map, finished, pqueue, cur, -1, 1, DIAG_STEP, 'q');
	// 	}

	// 	//add forward right location at next time
	// 	if(cur.dir == 'u' || cur.dir == 'e'
	// 		//if certain neighbors are blocked, maybe include
	// 		|| !valid(get_occ(map, cur.pt.x + 1, cur.pt.y, cur.pt.t)) && cur.dir == 'f'
	// 		|| !valid(get_occ(map, cur.pt.x, cur.pt.y + 1, cur.pt.t)) && cur.dir == 'r'){

	// 		add_neighbor(map, finished, pqueue, cur, 1, 1, DIAG_STEP, 'e');
	// 	}

	// 	//add back left location at next time
	// 	if(cur.dir == 'u' || cur.dir == 'z'
	// 		//if certain neighbors are blocked, maybe include
	// 		|| !valid(get_occ(map, cur.pt.x - 1, cur.pt.y, cur.pt.t)) && cur.dir == 'b'
	// 		|| !valid(get_occ(map, cur.pt.x, cur.pt.y - 1, cur.pt.t)) && cur.dir == 'l'){

	// 		add_neighbor(map, finished, pqueue, cur, -1, -1, DIAG_STEP, 'z');
	// 	}

	// 	//add back right location at next time
	// 	if(cur.dir == 'u' || cur.dir == 'c'
	// 		//if certain neighbors are blocked, maybe include
	// 		|| !valid(get_occ(map, cur.pt.x + 1, cur.pt.y, cur.pt.t)) && cur.dir == 'b'
	// 		|| !valid(get_occ(map, cur.pt.x, cur.pt.y - 1, cur.pt.t)) && cur.dir == 'r'){

	// 		add_neighbor(map, finished, pqueue, cur, 1, -1, DIAG_STEP, 'c');
	// 	}
	// }

		void AStar::inflate_map(timemap_server::TimeLapseMap &map){
		std::priority_queue<CellData> inflation_queue;

		bool* seen = new bool[size_x_ * size_y_];

		unsigned int min_i = 0;
		unsigned int min_j = 0;
		unsigned int max_i = size_x_;
		unsigned int max_j = size_y_;

		for(unsigned int k = 0; k < map.tmap.size(); k++){
			unsigned char* master_array = (unsigned char*)map.tmap[k].map.data.data();

			memset(seen, false, size_x_ * size_y_ * sizeof(bool));

			for (int j = min_j; j < max_j; j++)
			{
				for (int i = min_i; i < max_i; i++)
				{
					int index = size_y_ * j + i;
					unsigned char cost = master_array[index];
					if (cost == LETHAL_COST)
					{
						enqueue(master_array, seen, index, inflation_queue, i, j, i, j);
					}
				}
			}

			while (!inflation_queue.empty())
			{	
				//get the highest priority cell and pop it off the priority queue
				const CellData& current_cell = inflation_queue.top();
				unsigned int index = current_cell.index_;
				unsigned int mx = current_cell.x_;
				unsigned int my = current_cell.y_;
				unsigned int sx = current_cell.src_x_;
				unsigned int sy = current_cell.src_y_;
				//pop once we have our cell info
				inflation_queue.pop();
				//attempt to put the neighbors of the current cell onto the queue
				if (mx > 0){
					if(enqueue(master_array, seen, index - 1, inflation_queue, mx - 1, my, sx, sy)){
						pcl::PointXYZ pt;

						mapToWorld(mx, my, pt.x, pt.y);
						pt.z = (0.1) * (map.tmap[k].begin.toSec() - start_time_);

						inflation_.push_back(pt);
					}
				}
				if (my > 0){
					if(enqueue(master_array, seen, index - size_x_, inflation_queue, mx, my - 1, sx, sy)){
						pcl::PointXYZ pt;

						mapToWorld(mx, my, pt.x, pt.y);
						pt.z = (0.1) * (map.tmap[k].begin.toSec() - start_time_);

						inflation_.push_back(pt);
					}
				}
				if (mx < size_x_ - 1){
					if(enqueue(master_array, seen, index + 1, inflation_queue, mx + 1, my, sx, sy)){
						pcl::PointXYZ pt;

						mapToWorld(mx, my, pt.x, pt.y);
						pt.z = (0.1) * (map.tmap[k].begin.toSec() - start_time_);

						inflation_.push_back(pt);
					}
				}
				if (my < size_y_ - 1){
					if(enqueue(master_array, seen, index + size_x_, inflation_queue, mx, my + 1, sx, sy)){
						pcl::PointXYZ pt;

						mapToWorld(mx, my, pt.x, pt.y);
						pt.z = (0.1) * (map.tmap[k].begin.toSec() - start_time_);

						inflation_.push_back(pt);
					}
				}
			}	
		}
	}

	inline bool AStar::enqueue(unsigned char* grid, bool* seen, unsigned int index, std::priority_queue<CellData> &inflation_queue, unsigned int mx, unsigned int my,
		unsigned int src_x, unsigned int src_y)
	{
		//set the cost of the cell being inserted
		if (!seen[index])
		{
			// ROS_DEBUG("inflating...");
			//we compute our distance table one cell further than the inflation radius dictates so we can make the check below
			double distance = computeDistance(mx, my, src_x, src_y);
			//we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
			if (distance > cell_inflation_radius_)
			{
				return false;
			}

			//assign the cost associated with the distance from an obstacle to the cell
			unsigned char cost = computeCost(distance);
			unsigned char old_cost = grid[index];
			
			// ROS_DEBUG("updating cost");


			if (old_cost == UNKNOWN_COST && cost >= LETHAL_COST)
			{
				grid[index] = cost;
			}
			else
			{
				grid[index] = std::max(old_cost, cost);
			}

			//push the cell data onto the queue and mark
			seen[index] = true;
			CellData data(distance, index, mx, my, src_x, src_y);
			inflation_queue.push(data);
			
			return true;
		}
		return false;
	}

	double AStar::computeDistance(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1){
		return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
	}

	unsigned char AStar::computeCost(double distance){
		if(distance <= cell_inflation_radius_){
			return LETHAL_COST;
		}
		else{
			return OPEN_COST;
		}
	}

	inline void AStar::add_neighbor(const timemap_server::TimeLapseMap &map, const std::vector< std::deque< std::deque< Node > > > &finished, std::vector<Node> &pqueue, Node cur, int dx, int dy, int dt, char dir){
		Node new_node;

		new_node.pt.x   = cur.pt.x + dx;
		new_node.pt.y   = cur.pt.y + dy;
		new_node.pt.t   = cur.pt.t + dt;
		new_node.prev.t = cur.pt.t;
		new_node.prev.x = cur.pt.x;
		new_node.prev.y = cur.pt.y;
		new_node.dir    = dir;

		//if the corresponding cell is traversable
		if(valid(get_occ(map, new_node.pt.x, new_node.pt.y, new_node.pt.t))){
			//if we haven't already processed the node
			if(!finished_node(finished, new_node)){
				add_node(pqueue, new_node);
			}
		}
	}

	inline void AStar::add_node(std::vector<Node> &pqueue, Node node){
		for(int i=0; i < pqueue.size(); i++){
			//update time and prev if new node is better than old
			if(pqueue[i] == node){
				if(pqueue[i].pt.t > node.pt.t){
					pqueue[i].prev.x = node.prev.x;
					pqueue[i].prev.y = node.prev.y;
					pqueue[i].prev.t = node.prev.t;
					pqueue[i].dir    = node.dir;
					pqueue[i].pt.t   = node.pt.t;
				}
				return;
			} 
		}

		pqueue.push_back(node);
		std::push_heap(pqueue.begin(), pqueue.end(), CompareNodesHeuristic(goal_));
	}

	inline bool AStar::valid(int val){
		//this comment makes sublime happy...
		return (val != UNKNOWN_COST) && (val < LETHAL_COST);
	}

	inline double AStar::get_occ(const timemap_server::TimeLapseMap &map, int x, int y, int t){
		//find map that corresponds to the correct time
		int i=0;
		while(map.tmap[i].end.toSec() < t * time_res_ + start_time_){
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

	inline void AStar::add_finished(std::vector< std::deque< std::deque< Node > > > &finished, Node cur){
		int index_x, index_y;

		//find the correct time and allocate memory

		//longest time seen yet
		if(cur.pt.t + 1 > finished.size()){
			finished.resize(cur.pt.t + 1);
		}

		//time that has not been used yet
		if(finished[cur.pt.t].size() == 0){
			std::deque< Node > vy;
			vy.push_back(cur);
			finished[cur.pt.t].push_back(vy);

			return;
		}


		//find correct x and allocate memory

		//need to add row to front of deque
		if(cur.pt.x < finished[cur.pt.t][0][0].pt.x){
			std::deque< Node > temp;
			for(int i = finished[cur.pt.t][0][0].pt.x - cur.pt.x; i > 0; i--){
				finished[cur.pt.t].push_front(temp);
			}
			index_x = 0;
		}

		//need to add row to back of deque
		else if(cur.pt.x >= finished[cur.pt.t][0][0].pt.x + finished[cur.pt.t].size()){
			finished[cur.pt.t].resize(cur.pt.x - finished[cur.pt.t][0][0].pt.x + 1);
			index_x = cur.pt.x - finished[cur.pt.t][0][0].pt.x;
		}
		
		//row is already present...
		else{
			index_x = cur.pt.x - finished[cur.pt.t][0][0].pt.x;
		}
		

		//find correct y and allocate memory

		//row that has not been used yet
		if(finished[cur.pt.t][index_x].size() == 0){
			finished[cur.pt.t][index_x].push_back(cur);

			return;
		}

		//need to add node to front of deque
		if(cur.pt.y < finished[cur.pt.t][index_x][0].pt.y){
			Node temp;
			for(int i = finished[cur.pt.t][index_x][0].pt.y - cur.pt.y; i > 0; i--){
				finished[cur.pt.t][index_x].push_front(temp);
			}
			index_y = 0;
		}
		
		//need to add node to back of deque
		else if(cur.pt.y >= finished[cur.pt.t][index_x][0].pt.y + finished[cur.pt.t][index_x].size()){
			finished[cur.pt.t][index_x].resize(cur.pt.y - finished[cur.pt.t][index_x][0].pt.y + 1);
			index_y = cur.pt.y - finished[cur.pt.t][index_x][0].pt.y;
		}

		//need to add node to middle of deque
		else{
			index_y = cur.pt.y - finished[cur.pt.t][index_x][0].pt.y;
		}
		
		//add node
		finished[cur.pt.t][index_x][index_y].pt.x   = cur.pt.x;
		finished[cur.pt.t][index_x][index_y].pt.y   = cur.pt.y;
		finished[cur.pt.t][index_x][index_y].pt.t   = cur.pt.t;
		finished[cur.pt.t][index_x][index_y].prev.x = cur.prev.x;
		finished[cur.pt.t][index_x][index_y].prev.y = cur.prev.y;
		finished[cur.pt.t][index_x][index_y].prev.t = cur.prev.t;
		finished[cur.pt.t][index_x][index_y].dir    = cur.dir;
	}

	inline bool AStar::finished_node(const std::vector< std::deque< std::deque< Node > > > &finished, Node node){
		int index_x, index_y;

		//outside of time bounds
		if(finished.size() <= node.pt.t){
			return false;
		}

		//within time bounds but no nodes have that specific time...
		if(finished[node.pt.t].size() == 0){
			return false;
		}

		//outside of x bounds
		if(node.pt.x < finished[node.pt.t][0][0].pt.x){
			return false;
		}		

		if(node.pt.x >= finished[node.pt.t][0][0].pt.x + finished[node.pt.t].size()){
			return false;
		}

		index_x = node.pt.x - finished[node.pt.t][0][0].pt.x;

		//no nodes for given x value
		if(finished[node.pt.t][index_x].size() == 0){
			return false;
		}

		//outside of y bounds
		if(node.pt.y < finished[node.pt.t][index_x][0].pt.y){
			return false;
		}

		if(node.pt.y >= finished[node.pt.t][index_x][0].pt.y + finished[node.pt.t][index_x].size()){
			return false;
		}

		index_y = node.pt.y - finished[node.pt.t][index_x][0].pt.y;

		//node is default (all values default to 0)
		if(finished[node.pt.t][index_x][index_y].pt.t == finished[node.pt.t][index_x][index_y].prev.t){
			return false;
		}

		return true;
	}

	inline bool AStar::is_start(const Node node){
		//this comment makes sublime happy...
		return node.prev.t == -1; 
	}

	inline Node AStar::get_prev(const std::vector< std::deque< std::deque< Node > > > &finished, Node cur){
		int index_x = cur.prev.x - finished[cur.prev.t][0][0].pt.x;
		int index_y = cur.prev.y - finished[cur.prev.t][index_x][0].pt.y;

		return finished[cur.prev.t][index_x][index_y];
	}

	bool AStar::get_path(nav_msgs::Path &path, Node goal, const std::vector< std::deque< std::deque< Node > > > &finished){
		Node cur;

		cur = goal;

		//traceback through the path from goal to start
		while(!is_start(cur)){
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = ros::Time::now();
			pose.header.frame_id = "/map";

			mapToWorld(cur.pt.x, cur.pt.y, pose.pose.position.x, pose.pose.position.y);

			pose.pose.position.z = (0.1) * cur.pt.t * time_res_;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;

			path.poses.push_back(pose);

			cur = get_prev(finished, cur);
		}

		smooth_path(path);

		return !path.poses.empty();
	}

	void AStar::smooth_path(nav_msgs::Path &path){
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

	void AStar::publish_path(nav_msgs::Path &path){
		path.header.frame_id = path.poses[0].header.frame_id;
		path.header.stamp    = path.poses[0].header.stamp;

		path_pub_.publish(path);
	}

	//these functions are from the nav stack...
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