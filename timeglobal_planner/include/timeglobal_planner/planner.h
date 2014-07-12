#ifndef PLANNER_H
#define PLANNER_H

#include "timemap_server/TimeLapseMap.h"
#include "timemap_server/TimeLapseOccGrid.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <algorithm>
#include <vector>
#include <queue>

// Steps in time required

//TODO: params!!!

#define NORM_STEP    2
#define DIAG_STEP    3
#define TIME_STEP    1

#define LETHAL_COST  100
#define OPEN_COST    0
#define UNKNOWN_COST -1

#define GRID_X       10
#define GRID_Y       10
#define GRID_T       10

std::string global_frame = "map";
int end_time = 100;

namespace timeglobal_planner
{
struct Point {
	int x;
	int y;
	int t;
};

struct Node {
	Point pt;
	Node* prev;
};

bool operator==(const Node& lhs, const Node& rhs){return lhs.pt.x == rhs.pt.x && lhs.pt.y == rhs.pt.y;}

class CompareNodes {
	public:
		bool const operator()(const Node &lhs, const Node &rhs) {return (lhs.pt.t > rhs.pt.t);}
};

//path planning algorithm... hopefully
//returns false if path is trivial (start=end) or if no path is possible in timeframe
bool dijkstra(const timemap_server::TimeLapseMap &map, nav_msgs::Path &path, Point start_pt, Point goal_pt);

//will update dists as neighbors are added. neighbors that are added will have prev set to cur
//note: neighbors are in time levels below cur based on the values of their respective movement
void add_neighbors(const timemap_server::TimeLapseMap &map, const std::vector<Node> &finished, std::vector<Node> &pqueue, Node cur);

//as implied..
void add_neighbor(const timemap_server::TimeLapseMap &map, const std::vector<Node> &finished, std::vector<Node> &pqueue, Node cur, int dx, int dy, int dt);

//adds node to neighbors. if already present, updates dist
void add_node(std::vector<Node> &pqueue, Node node);

//checks if occ value is acceptable
inline bool valid(int val){return (val != UNKNOWN_COST) && (val < LETHAL_COST);}

//get the value for occupancy at a given location
double get_occ(const timemap_server::TimeLapseMap &map, int x, int y, int t);

//traces back the path and returns a list of waypoints
bool get_path(nav_msgs::Path &path, Node goal);

//checks if node has been fully processed
bool finished_node(const std::vector<Node> &finished, Node node);
}
#endif