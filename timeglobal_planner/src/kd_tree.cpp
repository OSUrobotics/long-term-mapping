#include "../include/kd_tree.h"

KDTree::KDTree(Point pt){
	root = 0;
	KDNode* new_root = new KDNode;

	new_root->val[0] = pt.x;
	new_root->val[1] = pt.y;
	// new_root->val[2] = pt.t;

	// printf("constructor: 1\n");
	insert(new_root);
	// printf("constructor: 2\n");

}

void KDTree::insert(KDNode* new_node){

	root = insert(root, new_node, 0);

	size++;
}

KDNode* KDTree::insert(KDNode *parent, KDNode *new_node, int depth){
	// printf("insert: 1\n");

	if(parent != 0){
	// printf("insert: 2\n");

		int axis = depth % DIM;
	// printf("insert: 3\n");

		if(parent->val[axis] < new_node->val[axis]){
	// printf("insert: 4a\n");

			parent->left = insert(parent->left, new_node, depth++);
		}
		else{
	// printf("insert: 4b\n");

			parent->right = insert(parent->right, new_node, depth++);
		}
	// printf("insert: 5\n");

	}
	else{
		parent = new_node;
	// printf("insert: 6\n");

	}
	// printf("insert: 7\n");

	return parent;
}

bool KDTree::balance(){return true;}

KDNode* KDTree::find_nearest(KDNode target){
	double min_dist;
	KDNode* cur;
	KDNode* nearest;

	cur = root;
	
	min_dist   = distance(*root, target);
	nearest    = root;

	find_nearest(&nearest, target, root, min_dist, 0);

	return nearest;
}

void KDTree::find_nearest(KDNode** nearest, const KDNode &target, KDNode *cur, double min_dist, int depth){
	int axis;
	double dist;

	if(cur != 0){
		axis = depth % DIM;
		dist = distance(*cur, target);

		// found new min
		if(min_dist > dist){
			min_dist = dist;
			*nearest = cur;

			// look at left first
			find_nearest(nearest, target, cur->left, min_dist, depth++);

			// min_dist might have changed so we may
			// not have to look at the right points
			if(cur->val[axis] - target.val[axis] > -min_dist){
				find_nearest(nearest, target, cur->right, min_dist, depth++);
			}
		}
		// may be able to eliminate points
		else{
			// we need to look at the left points
			if(cur->val[axis] - target.val[axis] < min_dist){
				find_nearest(nearest, target, cur->left, min_dist, depth++);
			}
			// we need to look at the right points
			if(cur->val[axis] - target.val[axis] > -min_dist){
				find_nearest(nearest, target, cur->right, min_dist, depth++);
			}
		}
	}
}

double KDTree::distance(const KDNode &first, const KDNode &second){
	double temp = 0;

	for(int i = 0; i < DIM; i++){
		temp += (first.val[i] - second.val[i]) * (first.val[i] - second.val[i]);
	}

	return sqrt(temp);
}

bool KDTree::get_display(nav_msgs::Path &path){
	return get_display(root, path);
}

bool KDTree::get_display(KDNode* parent, nav_msgs::Path &path){
	if(parent != 0){
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";

		pose.pose.position.x = parent->val[0];
		pose.pose.position.y = parent->val[1];
		pose.pose.position.z = (0.01) * parent->val[2];

		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;

		path.poses.push_back(pose);

		if(get_display(parent->left, path)){
			path.poses.push_back(pose);
		}

		if(get_display(parent->right, path)){
			path.poses.push_back(pose);
		}

		return true;
	}
	else{
		return false;
	}
}