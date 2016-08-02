#pragma warning (disable : 4267)

#include "GPU_Swarm.h"
#include <time.h>
#include <numeric>
#include <limits>

#include "cuda.h"
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include "device_launch_parameters.h"

#include "thrust/device_ptr.h"
#include "thrust/device_malloc.h"
#include "thrust/device_free.h"

#include "thrust/copy.h"
#include "thrust/fill.h"
#include "thrust/sequence.h"
#include "thrust/sort.h"
#include "thrust/extrema.h"

__global__ void Test_Kernel(float * inGraph, DPSO::Particle * p)
{
	int particleID = threadIdx.x + blockIdx.x*blockDim.x;
	
	DPSO::Particle particle = p[particleID];
	int offsetID = particle.graphOffset;

	p[particleID].psoResult = offsetID;
}

__global__ void CalculateValue_Kernel(float * inGraph, DPSO::Particle * p)
{
	int pID = threadIdx.x + blockIdx.x*blockDim.x;
	DPSO::Particle * _par = &p[pID];
	
	int offsetID = _par->graphOffset;
	int nodeCount = _par->positionSize;
	
	//thrust::device_ptr<int> dev_ptr = thrust::device_pointer_cast(_par->positionData);
	
	int _psoValue = 0;
	for (int i = 0; i < nodeCount-1; i++)
	{
		int fromNodeID = _par->positionData[i];
		int toNodeID = _par->positionData[i+1];
		float dist = inGraph[offsetID + fromNodeID*nodeCount + toNodeID];
		_psoValue += dist;
	}
	//Calculate last node
	int fromNodeID = _par->positionData[nodeCount-1];
	int toNodeID = _par->positionData[0];
	float dist = inGraph[offsetID + fromNodeID*nodeCount + toNodeID];
	_psoValue += dist;

	_par->psoResult = _psoValue;

	//Update Local-best
	if (_psoValue < _par->bestValue )
	{
		_par->bestValue = _psoValue;
	}
}

namespace DPSO
{
	SwarmCuda::SwarmCuda(int particle_count, float self_trust, float past_trust, float global_trust)
	{
		this->particle_count = particle_count;
		this->self_trust = self_trust;
		this->past_trust = past_trust;
		this->global_trust = global_trust;
		srand(int(time(NULL)));
	}

	void SwarmCuda::read_graph_definition(std::string filename)
	{
		read_graph_files(filename, graphCPU);
		loadGraphToGPU(graphCPU, graphGPU, this->particle_count);
	}

	void SwarmCuda::assign_particle_positions()
	{
		int gSize = graphGPU.num_nodes * graphGPU.num_edgesPerNode;
		int pSize = graphGPU.num_edgesPerNode;
		
		position_Sink.resize(pSize * particle_count);
		best_position_Sink.resize(pSize * particle_count);
		
		best_value = -1;
		best_position.resize(graphGPU.num_nodes);

		for (size_t i = 0; i < particle_count; i++)
		{
			//thrust::fill(position_Sink.begin() + i*pSize, position_Sink.begin() + (i + 1)*pSize, i);
			
			Particle _p;
			_p.pIndex = i;
			_p.graphOffset = i*gSize;
			_p.positionSize = pSize;
			_p.positionOffset = i*pSize;
			_p.psoResult = -1;
			_p.bestValue = std::numeric_limits<float>::max();

			_p.self_trust = this->self_trust;
			_p.past_trust = this->past_trust;
			_p.global_trust = this->global_trust;

			thrust::device_ptr<int> dev_ptr = &position_Sink[i*pSize];
			_p.positionData = thrust::raw_pointer_cast(dev_ptr);
			
			std::vector<int> newPos = shuffle();
			thrust::copy(newPos.begin(), newPos.end(), dev_ptr);

			gpuParticles.push_back(_p);
		}

		/*for (int i = 0; i < particle_count; i++)
		{
			Particle _p = gpuParticles[i];
			thrust::device_ptr<int> dev_ptr(_p.positionData);
			thrust::device_vector<int> dev_vec(dev_ptr, dev_ptr + pSize);

			std::cout << "Particle : " << i << std::endl;
			for (int j = 0; j < pSize; j++)
			{
				std::cout << dev_vec[j] << " ";
			}
			std::cout << std::endl;
		}*/

		std::cout << "Assigned! \n";

		int threadsPerBlock = particle_count;
		float * raw_graph_ptr = thrust::raw_pointer_cast(graphGPU.graphData.data());
		DPSO::Particle * parts = thrust::raw_pointer_cast(gpuParticles.data());
		CalculateValue_Kernel <<< 1, threadsPerBlock >> >(raw_graph_ptr, parts);
		std::cout << "First iteration \n";

		/// - TRACING
		for (int j = 0; j < particle_count; j++)
		{
			Particle _p = gpuParticles[j];
			std::cout << "P0[" << j << "] = " << _p.psoResult << std::endl;
		}

		/// - Can be replace with shared memory in Kernel ???
		int64_t minEle = thrust::min_element(gpuParticles.begin(), gpuParticles.end()) - gpuParticles.begin();
		Particle tPar = gpuParticles[minEle];

		thrust::copy(position_Sink.begin() + tPar.positionOffset,
			position_Sink.begin() + tPar.positionOffset + tPar.positionSize,
			best_position.begin());

		std::cout << "Min element P[" << minEle << "] = " << tPar.bestValue << std::endl;
	}

	std::vector<int> SwarmCuda::shuffle()
	{
		/// shuffle 0->NodeSize
		//Knuth-Fisher-Yates shuffle
		int pSize = graphGPU.num_nodes;
		std::vector<int> newPos(pSize);
		std::iota(newPos.begin(), newPos.end(), 0);

		for (int i = (int)newPos.size() - 1; i > 0; i--)
		{
			int n = rand() % (i + 1);
			int tmp = newPos[i];
			newPos[i] = newPos[n];
			newPos[n] = tmp;
		}
		return newPos;
	}

	void SwarmCuda::testSwarmAction()
	{
		std::cout << "Testing ! \n" << std::endl;
		int threadsPerBlock = this->particle_count;
		int blocksPerGrid = 1;

		float * raw_graph_ptr = thrust::raw_pointer_cast(graphGPU.graphData.data());

		DPSO::Particle * parts = thrust::raw_pointer_cast(gpuParticles.data());
		Test_Kernel <<< blocksPerGrid, threadsPerBlock >>>(raw_graph_ptr, parts);

		for (int i = 0; i < gpuParticles.size();i++)
		{
			DPSO::Particle p = gpuParticles[i];
			std::cout << "R[" << i << "] = " << p.psoResult << std::endl;
		}

		//size_t max_ele = thrust::max_element(gpuParticles.begin(), gpuParticles.end()) - gpuParticles.begin();
		//std::cout << "Max element = " << max_ele << std::endl;
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
	graphGPU.num_nodes = (int)graphCPU.size();
	graphGPU.num_edgesPerNode = (int)graphCPU.size();
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

	/// - Load graph to Thrust vector
	graphGPU.graphData.clear();
	graphGPU.graphData.resize(partSize*numParticles);
	for (int i = 0; i < numParticles; i++)
	{
		thrust::copy(fullData.begin(), fullData.end(),
			graphGPU.graphData.begin() + i*partSize);
	}

	/*for (int n = 0; n < numParticles; n++)
	{
		int part_offset = n*partSize;
		std::cout << "NODE = " << n << ": \n";
		for (int i = 0; i < graphGPU.num_nodes; i++)
		{
			for (int j = 0; j < graphGPU.num_edgesPerNode; j++)
			{
				int idX = part_offset + i*graphGPU.num_edgesPerNode + j;
				std::cout << graphGPU.graphData[idX] << " ";
			}
			std::cout << std::endl;
		}
	}*/
}