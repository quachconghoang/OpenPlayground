#include "stdafx.h"
#include "Swarm.h"
#include <time.h>

Swarm::Swarm(int particle_count, float self_trust, float past_trust, float global_trust){
	this->particle_count = particle_count;
	this->self_trust = self_trust;
	this->past_trust = past_trust;
	this->global_trust = global_trust;
	srand(unsigned int(time(NULL)));
}

double Swarm::solve(){
	int moves_since_best_changed = 0;

	// if the cost does not change after 4 times -> finished
	while(moves_since_best_changed <= 4){
		bool best_changed = false;
		
		if( moves_since_best_changed < 4 ){  // if cost changing steadily
			best_changed = normal_search();
		}

		if(!best_changed){
			moves_since_best_changed++;
		}else{
			moves_since_best_changed = 0;
		}
		std::cout << "Best value so far: " << this->best_value << std::endl;
	}
	return this->best_value;
}

// Normal update of particles
bool Swarm::normal_search(){
	bool best_changed = false;
	double tmp;
	for(int i = 0; i < this->particles.size(); i++){
		//std::cout << i << std::endl;
		this->particles[i].calculate_new_velocity(this->best_position);
		tmp = this->particles[i].move();
		if(this->best_value > tmp){ // update best cost and position
			this->best_value = tmp;
			this->best_position = this->particles[i].position;
			best_changed = true;
		}
	}

	return best_changed;
}

void Swarm::particles_back_to_best()
{
	for(int i = 0; i < this->particles.size(); i++)
	{
		this->particles[i].position = this->particles[i].best_position;
	}
}

void Swarm::read_graph_definition(std::string filename){
	
	std::ifstream graph_file;
	this->nodes.clear();

	graph_file.open(filename.c_str());
	if (graph_file.is_open()){

		int totalNode;
		graph_file >> totalNode;
		std::cout << "Reading nodes:\n";

		for (int i = 0; i < totalNode;i++)
		{
			GpuNode n;
			n.cost_to.resize(totalNode);
			graph_file >> n.index;
			graph_file >> n.x;
			graph_file >> n.y;
			graph_file >> n.z;

			for (int j = 0; j < totalNode;j++)
			{
				graph_file >> n.cost_to[j];
			}
			this->nodes.push_back(n);
		}
		graph_file.close();
	}
	else
	{
		std::cout << "Could not open file" << std::endl;
		throw(-1);
	}

	assign_particle_positions();
}

void Swarm::assign_particle_positions()
{
	this->particles.clear();
	for(int i = 0; i < this->particle_count; i++ )
	{	
		this->particles.push_back( Particle(this->self_trust, this->past_trust, this->global_trust) );
		this->particles[i].position = shuffle(); // add a random position (a random path) for a particle
		double cur_value = this->particles[i].calculate_value();

		if(i==0 || this->best_value > cur_value)
		{
			this->best_value = cur_value;
			this->best_position = this->particles[i].position;
		}
	}
}

// Randomize nodes of a position
Position Swarm::shuffle()
{
	Position p;
	std::vector<GpuNode> new_vec(this->nodes); // this->nodes: nodes obtained from the input file

	//Knuth-Fisher-Yates shuffle
	for(int i = (int)new_vec.size() - 1; i > 0; i--)
	{
		int n = rand() % (i + 1);
		GpuNode tmp = new_vec[i];
		new_vec[i] = new_vec[n];
		new_vec[n] = tmp;
	}

	p.nodes = new_vec;
	return p;
}

std::string Swarm::trim(std::string s)
{
	s.erase(s.find_last_not_of(" \n\r\t") + 1);

	return s;
}
