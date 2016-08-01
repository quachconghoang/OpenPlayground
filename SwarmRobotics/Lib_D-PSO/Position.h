#include "velocity.h"
#include <vector>
#include <string>
#include <sstream>
#include "Eigen/Dense"

#ifndef POSITION
#define POSITION


//class Node {
//public:
//	Node(int _index, double _x, double _y, double _z);
//	bool operator==(const Node &rhs);
//
//	int index;
//	double x, y, z;
//	std::vector<double> cost_to;
//	std::vector<Eigen::Vector3i> path_to;
//
//	double distance_to(const Node &other);
//	Node();
//};

struct GpuNode
{
	int index;
	int x, y, z;
	std::vector<double> cost_to;
};

// A position is an array of nodes representing the vertices that salesman need to go through alternatively
class Position{
public:
	Position();
	Position(const Position &p);
	Position& operator=(const Position & rhs);
	Position& operator+=(const Velocity & rhs);
	Position operator+(const Velocity & rhs);
	Velocity operator-(const Position &p);
	Position& add_node(GpuNode new_n);
	std::string to_string();
    
	std::vector<GpuNode> nodes;
    
};

#endif
