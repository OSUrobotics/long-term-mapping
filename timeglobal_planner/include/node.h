#ifndef NODE_H
#define NODE_H

struct Point {
	int x;
	int y;
	int t;
};

// struct Node {
// 	Point pt;
// 	int prev;
// };

struct Node {
	Point pt;
	Point prev;

	//  q f e
	//  l u r
	//  z b c
	char dir;
};

bool operator==(const Node& lhs, const Node& rhs){return lhs.pt.x == rhs.pt.x && lhs.pt.y == rhs.pt.y;}

#endif