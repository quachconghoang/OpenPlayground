//**************************************//
#include "stdafx.h"
#include "SearchNode.h"

Node::Node() {
	this->index = -1;
	this->x = 0;
	this->y = 0;
	this->z = 0;
}

Node::Node(int _index, double _x, double _y, double _z) {
	this->index = _index;
	this->x = _x;
	this->y = _y;
	this->y = _z;
}

bool Node::operator==(const Node &rhs) {
	if (this->index == rhs.index) {
		return true;
	}
	return false;
}

double Node::distance_to(const Node &other) {
	int x = this->x - other.x;
	int y = this->y - other.y;
	int z = this->z - other.z;
	double val = std::sqrt((x*x) + (y*y) + (z*z));
	return val;
}


SearchNode::SearchNode(Point3DInt *position, double cost, double pathCost, SearchNode *next)
{
	this->position = *position;
	this->cost = cost;
	this->pathCost = pathCost;
	this->next = next;
}
