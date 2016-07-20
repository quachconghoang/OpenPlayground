#pragma once

#include "World.h"

class Node {
public:
	Node(int _index, double _x, double _y, double _z);
	bool operator==(const Node &rhs);

	int index;
	double x, y, z;
	std::vector<double> cost_to;
	std::vector<Point3DInt> path_to;

	double distance_to(const Node &other);
	Node();
};


class SearchNode
{
public:
	SearchNode(Point3DInt *position, double cost, double pathCost, SearchNode *next);
	Point3DInt position;
	double cost;
	double pathCost;
	SearchNode *next;
};

