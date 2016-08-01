#ifndef GPU_SWARM_H
#define GPU_SWARM_H

#include <iostream>
#include <fstream>
#include <vector>

#include "GPU_Particle.h"

#include "thrust/host_vector.h"
#include "thrust/device_vector.h"

struct GraphNode
{
	int index;
	int x, y, z;
	std::vector<float> cost_to;
};

struct GraphGPU
{
	int num_particles;
	int num_nodes;
	int num_edgesPerNode;
	thrust::device_vector<float> graphData;
};

namespace DPSO
{
	class SwarmCuda
	{
	public:
		SwarmCuda(){};
		~SwarmCuda(){};

		SwarmCuda(int particle_count, float self_trust, float past_trust, float global_trust);

		void read_graph_definition(std::string filename);
		
		std::vector<GraphNode> graphCPU;
		GraphGPU graphGPU;

		double best_value;
		Position best_position;

	private:
		double self_trust;
		double past_trust;
		double global_trust;
		int particle_count;
	};
}

void read_graph_files(std::string filename, std::vector<GraphNode> & graphData);
void loadGraphToGPU(std::vector<GraphNode> & graphCPU, GraphGPU & graphGPU, int numParticles);

#endif