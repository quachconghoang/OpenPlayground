#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#include <cmath>

#ifndef VELOCITY
#define VELOCITY

// Each velocity has following properties:
// Value: the sequence of swap (e.g. swap to[i] with from[i] ) that need to be executed
// Size: number of swap to be executed
// Operators

class Velocity{
public:
    Velocity();  // Init zero velocity
	Velocity(const Velocity &v);   // Init v
	Velocity(std::vector<int> _from, std::vector<int> _to);  // Init from transposition sequences

	// Define the operator for velocity
    Velocity& operator=(const Velocity &rhs);
    Velocity& operator*=(const double &rhs);
    Velocity& operator+=(const Velocity &rhs); // add more swap (transposition) to the end of v
    Velocity operator*(const double &rhs);
    Velocity operator+(const Velocity &rhs); // remove swap from the end of

	// To display the content of velocity
	std::string to_string();

    void add_transposition(int a, int b);  // Add a pair of nodes that need to be swapped
    void remove_transposition(int index);

    int size;
    std::vector<int> from;   // Indices of node to swap
    std::vector<int> to;	//
};

#endif
