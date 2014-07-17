#include "../include/kd_tree.h"

void KDTree::insert(KDNode* new_node){
	root = insert(root, new_node, 0);

	size++;
}

KDNode* KDTree::insert(KDNode *parent, KDNode *new_node, int depth){
	if(parent != 0){
		int axis = depth % DIM;

		if(parent->val[axis] < new_node->val[axis]){
			parent->left = insert(parent->left, new_node, depth++);
		}
		else{
			parent->right = insert(parent->right, new_node, depth++);
		}
	}
	else{
		parent = new_node;
	}

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
