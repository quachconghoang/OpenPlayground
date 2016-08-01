#include "GPU_Swarm.h"
#include <time.h>

namespace DPSO
{
	SwarmCuda::SwarmCuda(int particle_count, float self_trust, float past_trust, float global_trust)
	{
		this->particle_count = particle_count;
		this->self_trust = self_trust;
		this->past_trust = past_trust;
		this->global_trust = global_trust;
		srand(unsigned int(time(NULL)));
	}

	void SwarmCuda::read_graph_definition(std::string filename)
	{
		read_graph_files(filename, graphCPU);
		loadGraphToGPU(graphCPU, graphGPU, this->particle_count);
	}
}

void read_graph_files(std::string filename, std::vector<GraphNode> & graphData)
{
	std::ifstream graph_file;
	graph_file.open(filename.c_str());
	if (graph_file.is_open()){
		int totalNode;
		graph_file >> totalNode;
		std::cout << "Reading nodes :\n";

		for (int i = 0; i < totalNode; i++)
		{
			GraphNode n;
			n.cost_to.resize(totalNode);
			graph_file >> n.index;
			graph_file >> n.x;	graph_file >> n.y;	graph_file >> n.z;

			for (int j = 0; j < totalNode; j++){
				graph_file >> n.cost_to[j];
			}
			graphData.push_back(n);
		}
		graph_file.close();
		std::cout << totalNode << " nodes were read ... \n";
	}
	else{
		std::cout << "Could not open file" << std::endl;	throw(-1);
	}
}

void loadGraphToGPU(std::vector<GraphNode> & graphCPU, GraphGPU & graphGPU, int numParticles)
{
	graphGPU.num_nodes = graphCPU.size();
	graphGPU.num_edgesPerNode = graphCPU.size();
	graphGPU.num_particles = numParticles;

	int edgeSize = graphGPU.num_edgesPerNode;
	int partSize = graphGPU.num_nodes * graphGPU.num_edgesPerNode;

	std::vector<float> fullData(partSize);
	for (int i = 0; i < graphCPU.size(); i++)
	{
		std::copy(graphCPU[i].cost_to.begin(),
			graphCPU[i].cost_to.end(),
			fullData.begin() + i*edgeSize);
	}

	//Load graph to Thrust data
	graphGPU.graphData.clear();
	graphGPU.graphData.resize(partSize*numParticles);
	for (int i = 0; i < numParticles; i++)
	{
		thrust::copy(fullData.begin(), fullData.end(),
			graphGPU.graphData.begin() + i*partSize);
	}

	//for (int n = 0; n < numParticles; n++)
	//{
	//	int part_offset = n*partSize;
	//	std::cout << "NODE = " << n << ": \n";
	//	for (int i = 0; i < graphGPU.num_nodes; i++)
	//	{
	//		for (int j = 0; j < graphGPU.num_edgesPerNode; j++)
	//		{
	//			int idX = part_offset + i*graphGPU.num_edgesPerNode + j;
	//			std::cout << graphGPU.graphData[idX] << " ";
	//		}
	//		std::cout << std::endl;
	//	}
	//}
}