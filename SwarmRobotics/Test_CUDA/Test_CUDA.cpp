// Test_CUDA.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <conio.h>
#include <iostream>
#include <fstream>
#include <vector>

//#include "GraphGPU.h"
#include "GPU_Swarm.h"

#define NUM_PARTICLES 64
using namespace DPSO;

int _tmain(int argc, _TCHAR* argv[])
{
	std::cout << "Running PSO in CUDA ..." << std::endl;
	int particle_count = NUM_PARTICLES;
	float self_trust = 0.f; // c1
	float past_trust = 0.5f; // parameter for personal best (c2)
	float global_trust = 0.5f; // parameter for global best (c3)

	SwarmCuda cuSwarm(particle_count, self_trust, past_trust, global_trust);
	cuSwarm.read_graph_definition("..//..//IndoorData//AstarGraph.txt");

	cuSwarm.assign_particle_positions();
	//cuSwarm.shuffle();
	//cuSwarm.testSwarmAction();
	//_getch();

	cuSwarm.move_particle();
	//cuSwarm.showParticleData();
	_getch();

	return 0;
}