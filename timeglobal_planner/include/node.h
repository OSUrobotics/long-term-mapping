#ifndef NODE_H
#define NODE_H

#include <deque>

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

struct Row {
	Row() : y(0){};
	int y;
	std::deque< char > data;
};

struct Layer {
	Layer() : x(0){};
	int x;
	std::deque< Row > rows;
};

bool operator==(const Point& lhs, const Point& rhs){return lhs.x == rhs.x && lhs.y == rhs.y;}

bool operator==(const Node& lhs, const Node& rhs){return lhs.pt.x == rhs.pt.x && lhs.pt.y == rhs.pt.y;}

#endif