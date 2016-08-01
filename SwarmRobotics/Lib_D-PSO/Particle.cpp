#include "stdafx.h"
#include "particle.h"

Particle::Particle(double self_trust,double past_trust,double global_trust){
	this->self_trust = self_trust;
	this->past_trust = past_trust;
	this->global_trust = global_trust;
	this->best_value = -1;
}

double Particle::move(){
//	std::cout << "Value before move: " << this->calculate_value() << std::endl;
    position += velocity;
//	std::cout << "Value after move: " << this->calculate_value() << std::endl;

	opt_improvement();

//	std::cout << "Value after Opt: " << this->calculate_value() << std::endl;

	double new_value = this->calculate_value();
	//std::cout << new_value << std::endl;
    if(new_value < this->best_value || this->best_value < 0){
        this->best_value = new_value;
		this->best_position = position;
    }
	
    return this->best_value;
}

double Particle::calculate_value(){
    double value = 0;

	int node_count = (int)this->position.nodes.size();

    for(int i = 0; i < node_count; i++){
        GpuNode tmp = this->position.nodes[i];
		GpuNode tmp2;

		if( i+1 < node_count ){  // not last node
			tmp2 = this->position.nodes[i+1];
		}else{
			tmp2 = this->position.nodes[0];  // The first and last node must be the same
		}
		double tmp_val = tmp.cost_to[tmp2.index];
		value += tmp_val;
    }

    return value;
}

void Particle::calculate_new_velocity(Position global_best){
	Velocity a; 
	if(this->velocity.size > 0){
		a = (this->velocity * this->self_trust); // self velocity component
	}

	Velocity b = ((this->best_position - this->position) * this->past_trust );  // personal best velocity component << lack of random coefficient
	Velocity c = ((global_best-this->position) * this->global_trust);		// global best component << lack of random coefficient

	//std::cout << a.to_string();
	this->velocity =  Velocity(a + b + c);
}

double Particle::opt_improvement() {
	double best_cost_change = 0;
	int i_best = -1;
	int j_best = -1;

	int node_count = (int)this->position.nodes.size();

	for (int i = 0; i < node_count - 2; i++){
		double edge_length_i = edge_length(i);

		for (int j = i+2; j < node_count; j++){
			double old_cost = edge_length_i + edge_length(j);
			double new_cost = edge_length_swap(i, j);
			double cost_change = new_cost - old_cost;
			
			if (cost_change < best_cost_change)	{
				best_cost_change = cost_change;
				i_best = i;
				j_best = j;
			}
		}
	}

//	std::cout << best_cost_change << std::endl;
//	std::cout << "Value before swap: " << this->calculate_value() << std::endl;
	swap_edge(i_best, j_best);
//	std::cout << "Value before after: " << this->calculate_value() << std::endl;

	return this->calculate_value();
}

// Calculate the length of edge connecting node[index] and node[index+1]
double Particle::edge_length(int i) {
	int next = (i == this->position.nodes.size() - 1) ? 0 : i + 1;  // If index = (node_number - 1)  -> last node -> next node is set to be the start node
	
	GpuNode current_node = this->position.nodes[i];
	GpuNode next_node = this->position.nodes[next];

	return current_node.cost_to[next_node.index];
}

double Particle::edge_length_swap(int i, int j) {
	int node_count = (int)this->position.nodes.size();
	
	int next_i = (i == node_count - 1) ? 0 : i + 1;  // If index = (node_number - 1)  -> last node -> next node is set to be the start node
	int next_j = (j == node_count - 1) ? 0 : j + 1;

	GpuNode current_node_i = this->position.nodes[i];
	GpuNode next_node_i = this->position.nodes[next_i];

	GpuNode current_node_j = this->position.nodes[j];
	GpuNode next_node_j = this->position.nodes[next_j];

	double edge_length_i = current_node_i.cost_to[current_node_j.index];
	double edge_length_j = next_node_i.cost_to[next_node_j.index];

	return (edge_length_i + edge_length_j);
}

void Particle::swap_edge(int i, int j) {
	Velocity v;
	int middle = (j - i)/2;
	for (int k = 0; k < middle; k++) {
		v.add_transposition(i + k + 1, j - k);
	}
	this->position += v;
}