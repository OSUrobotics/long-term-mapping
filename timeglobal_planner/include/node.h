#ifndef NODE_H
#define NODE_H

struct Point {
	Point() : x(0), y(0), t(0){};
	int x;
	int y;
	int t;
};

struct Node {
	Node() : dir('!'){};
	Point pt;
	Point prev;

	//  q f e
	//  l u r
	//  z b c
	char dir;
};

bool operator==(const Node& lhs, const Node& rhs){return lhs.pt.x == rhs.pt.x && lhs.pt.y == rhs.pt.y;}

#endif