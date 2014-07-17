#ifndef KD_TREE_H
#define KD_TREE_H

#include "node.h"

#include <vector>
#include <cmath>

#define DIM 3

struct KDNode {
	KDNode() : left(0), right(0), rtt_parent(0){};

	int val[DIM];

	KDNode* left;
	KDNode* right;
	KDNode* rtt_parent;
};

class KDTree {
public:
	KDTree(std::vector<KDNode> points);

	void insert(KDNode* new_node);

	bool balance();

	KDNode* find_nearest(KDNode target);

	int get_size(){return size;};

private:
	KDNode* root;

	int size;

	KDNode* insert(KDNode *parent, KDNode *new_node, int depth);

	void find_nearest(KDNode** nearest, const KDNode &target, KDNode *cur, double min_dist, int depth);

	double distance(const KDNode &first, const KDNode &second);


};

#endif