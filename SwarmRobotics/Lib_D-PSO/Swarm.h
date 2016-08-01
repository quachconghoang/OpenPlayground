#ifndef SWARM
#define SWARM

#include "particle.h"
//#include "node.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

class Swarm{
public:
	Swarm(int particle_count, float self_trust, float past_trust, float global_trust);
	
	void read_graph_definition(std::string filename);

	void assign_particle_positions();
	double solve();

    std::vector<Particle> particles;
	std::vector<GpuNode> nodes;
	double best_value;
	Position best_position;
private:
	bool normal_search();

	void particles_back_to_best();

	Swarm();//shouldn't be using this
	std::string trim(std::string);
	Position shuffle();

	double self_trust;
	double past_trust;
	double global_trust;
	int particle_count;
};

#endif
