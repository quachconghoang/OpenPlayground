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

#define NUM_PARTICLES 128
namespace DPSO
{
	class SwarmCuda
	{
	public:
		SwarmCuda(){};
		~SwarmCuda(){};

		SwarmCuda(int particle_count, float self_trust, float past_trust, float global_trust);

		void read_graph_definition(std::string filename);
		void assign_particle_positions();

		void solve();
		bool move_particle();

		std::vector<GraphNode> graphCPU;
		GraphGPU graphGPU;

		thrust::device_vector<Particle> gpuParticles;
		float best_value;
		int bestParticleNum;

		thrust::device_vector<int> position_Sink;
		thrust::device_vector<int> best_position_Sink;
		thrust::device_vector<D_Vec2i> velocity_Sink;

		void showParticleData();
	private:
		float self_trust;
		float past_trust;
		float global_trust;
		int particle_count;
		std::vector<int> shuffle();
	};
}

void read_graph_files(std::string filename, std::vector<GraphNode> & graphData);
void loadGraphToGPU(std::vector<GraphNode> & graphCPU, GraphGPU & graphGPU, int numParticles);

#endif