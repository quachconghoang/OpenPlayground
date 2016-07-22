#include "Velocity.h"
#include "Position.h"
#include <climits>

#ifndef PARTICLE
#define PARTICLE

class Particle{
public:
	Particle(double self_trust,double past_trust,double global_trust);

    double move();  //Move based on velocity, return best value seen
    double calculate_value();  //Get sum of weights of the edges we're using
	void calculate_new_velocity(Position global_best);
	double opt_improvement(); // Improve the particle using 2-opt algorithm

    Velocity velocity;
    Position position;
    Position best_found;
    double best_value;
	Position best_position;

	//These should add up to 1.0
	double self_trust;
	double past_trust;
	double global_trust;
private:
	double edge_length(int i);
	double edge_length_swap(int i, int j);
	void swap_edge(int i, int j);
};

#endif
