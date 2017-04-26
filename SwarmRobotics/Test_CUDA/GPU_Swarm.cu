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

#include "opencv2/opencv.hpp"

void checkCUDAError(const char *msg)
{
	cudaError_t err = cudaGetLastError();
	if (cudaSuccess != err)
	{
		fprintf(stderr, "Cuda error: %s: %s.\n", msg, cudaGetErrorString(err));
		exit(EXIT_FAILURE);
	}
}

__device__ void calculateValueParticle(DPSO::Particle * _par)
{
	int nodeCount = _par->positionSize;

	float _psoValue = 0;
	for (int i = 0; i < nodeCount - 1; i++)
	{
		int fromNodeID = _par->positionData[i];
		int toNodeID = _par->positionData[i + 1];
		float dist = _par->graphData[fromNodeID*nodeCount + toNodeID];
		_psoValue += dist;
	}
	//Calculate last node
	int fromNodeID = _par->positionData[nodeCount - 1];
	int toNodeID = _par->positionData[0];
	float dist = _par->graphData[fromNodeID*nodeCount + toNodeID];
	_psoValue += dist;

	_par->psoResult = _psoValue;

	//Update Local-best
	if (_psoValue < _par->bestValue)
	{
		_par->bestValue = _psoValue;
		for (int i = 0; i < _par->positionSize; i++)
		{
			_par->bestPosition[i] = _par->positionData[i];
		}
	}
}

__device__ float edge_length(int locID, int nodeCount, float * _graph)
{
	int next_locID = (locID == (nodeCount - 1)) ? 0 : locID + 1;
	return _graph[locID*nodeCount + next_locID];
}

__device__ float edge_length_swap(int i, int j, int nodeCount, float * _graph)
{
	int next_i = (i == nodeCount - 1) ? 0 : i + 1;  // If index = (node_number - 1)  -> last node -> next node is set to be the start node
	int next_j = (j == nodeCount - 1) ? 0 : j + 1;

	float edge_length_i = _graph[i*nodeCount + next_i];
	float edge_length_j = _graph[j*nodeCount + next_j];

	return (edge_length_i + edge_length_j);
}

__global__ void CalculateValue_Kernel(DPSO::Particle * p)
{
	int pID = threadIdx.x + blockIdx.x*blockDim.x;
	DPSO::Particle * _par = &p[pID];
	calculateValueParticle(_par);
}

__global__ void Moving_Kernel(DPSO::Particle * p, int bestParticleID)
{
	/// 0. Load best position to array
	__shared__ int best[1024];
	DPSO::Particle * _bestPar = &p[bestParticleID];
	for (int i = 0; i < _bestPar->positionSize;i++)
	{
		best[i] = _bestPar->bestPosition[i];
	}
	//int pcID = threadIdx.x + blockIdx.x*blockDim.x;
	//DPSO::Particle * _bestPar = &p[bestParticleID];
	//int positionSize = _bestPar->positionSize;
	//int step = 0;
	//int cpThreadLoc;
	//while ((cpThreadLoc = pcID + step*NUM_PARTICLES) < positionSize)
	//{
	//	best[cpThreadLoc] = _bestPar->bestPosition[cpThreadLoc];
	//	step++;
	//}
	__syncthreads();
	
	/// 1. Calculate new velocities from best-position & trust values (c2 & c3)
	int pID = threadIdx.x + blockIdx.x*blockDim.x;
	DPSO::Particle * _par = &p[pID];

	//calculate_Velocities(_par, &best[0]);

	int posSize = _par->positionSize;
	int _velocitySize = 0;

	// 1a. Get velocity from C2: past-trust
	int c2Size = ceilf(posSize *_par->past_trust);
	for (int i = 0; i < c2Size; i++)
	{
		int look_for_ID = _par->positionData[i];
		int found_at = -1;
		for (int j = 0; j < posSize; j++)
		{
			if (look_for_ID == _par->positionData[j])	{	found_at = j;	break;	}
		}

		if ((i != found_at) && (found_at != -1))
		{
			_par->velocity[_velocitySize].from = i;
			_par->velocity[_velocitySize].to = found_at;
			_velocitySize++;
		}
	}

	// 1b. Get velocity from C3: global-trust
	int v_offset = _velocitySize;
	int c3Size = ceilf(posSize*_par->global_trust);
	for (int i = 0; i < c3Size; i++)
	{
		int look_for_ID = _par->positionData[i];
		int found_at = -1;
		for (int j = 0; j < posSize; j++)
		{
			if (look_for_ID == best[j])	{ found_at = j;	break; }
		}

		if ((i != found_at) && (found_at != -1))
		{
			int vID = v_offset + i;
			_par->velocity[vID].from = i;
			_par->velocity[vID].to = found_at;
			_velocitySize++;
		}
	}

	_par->velocitySize = _velocitySize;
	//printf("Thread: %d - c1 velocity_size %d \n", pID, _velocitySize);

	/// 2. Calculate best position from new velocities
	//		2a. Check all velocity
	//		2b. Swap node positions
	for (int i = 0; i < _velocitySize; i++)
	{
		DPSO::D_Vec2i swapVal = _par->velocity[i];
		int tmp = _par->positionData[swapVal.from];
		_par->positionData[swapVal.from] = _par->positionData[swapVal.to];
		_par->positionData[swapVal.to] = tmp;
	}
	_par->velocitySize = 0;

	/// 3. Two opt
	float best_cost_change = 0;
	int i_best = -1;
	int j_best = -1;
	int node_count = _par->positionSize;

	for (int i = 0; i < node_count - 2; i++){
		float edge_length_i = edge_length(i,node_count,_par->graphData);

		for (int j = i + 2; j < node_count; j++){
			float old_cost = edge_length_i + edge_length(j, node_count, _par->graphData);;
			float new_cost = edge_length_swap(i, j, node_count, _par->graphData);
			float cost_change = new_cost - old_cost;

			if (cost_change < best_cost_change)	{
				best_cost_change = cost_change;
				i_best = i;
				j_best = j;
			}
		}
	}
	/// Swap edges;
	int middle = (j_best - i_best) / 2;
	for (int k = 0; k < middle; k++) {
		int _from = i_best + k + 1;
		int _to = j_best + k;
		
		/// Swapping
		int tmp = _par->positionData[_from];
		_par->positionData[_from] = _par->positionData[_to];
		_par->positionData[_to] = tmp;
	}

	/// 4. Calculate Values
	calculateValueParticle(_par);
}

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

	void SwarmCuda::assign_particle_positions()
	{
		int gSize = graphGPU.num_nodes * graphGPU.num_edgesPerNode;
		int pSize = graphGPU.num_nodes;
		
		position_Sink.resize(pSize * particle_count);
		best_position_Sink.resize(pSize * particle_count);
		velocity_Sink.resize(pSize*particle_count);

		best_value = -1;

		for (size_t i = 0; i < particle_count; i++)
		{	
			//thrust::fill(position_Sink.begin() + i*pSize, position_Sink.begin() + (i + 1)*pSize, -1);
			//thrust::fill(best_position_Sink.begin() + i*pSize, best_position_Sink.begin() + (i + 1)*pSize, -1);
			//DPSO::D_Vec2i v_i; v_i.from = 0; v_i.to = 0;
			//thrust::fill(velocity_Sink.begin() + i*pSize, velocity_Sink.begin() + (i + 1)*pSize, v_i);

			Particle _p;
			_p.pIndex = i;
			_p.graphOffset = i*gSize;
			_p.positionSize = pSize;
			_p.positionOffset = i*pSize;
			_p.psoResult = -1;
			_p.bestValue = std::numeric_limits<float>::max();
			_p.velocitySize = 0;

			_p.self_trust = this->self_trust;
			_p.past_trust = this->past_trust;
			_p.global_trust = this->global_trust;

			thrust::device_ptr<int> dev_ptr = &position_Sink[i*pSize];
			_p.positionData = thrust::raw_pointer_cast(dev_ptr);
			_p.bestPosition = thrust::raw_pointer_cast(&best_position_Sink[i*pSize]);
			_p.velocity = thrust::raw_pointer_cast(&velocity_Sink[i*pSize]);
			_p.graphData = thrust::raw_pointer_cast(&graphGPU.graphData[i*gSize]);
			
			std::vector<int> newPos = shuffle();
			thrust::copy(newPos.begin(), newPos.end(), dev_ptr);

			gpuParticles.push_back(_p);
		}

		//showParticleData();
		std::cout << "Assigned! \n";

		int threadsPerBlock = particle_count;
		DPSO::Particle * parts = thrust::raw_pointer_cast(gpuParticles.data());
		CalculateValue_Kernel <<< 1,threadsPerBlock >>>(parts);

		std::cout << "First iteration \n";
		bestParticleNum = int(thrust::min_element(gpuParticles.begin(), gpuParticles.end()) - gpuParticles.begin());
		Particle tPar = gpuParticles[bestParticleNum];
		best_value = tPar.bestValue;
		std::cout << " - P[" << bestParticleNum << "] = " << best_value << std::endl;

		//move_particle();
		//showParticleData();
	}

	bool SwarmCuda::move_particle()
	{
		//float previous_best = this->best_value;
		bool best_changed = false;

		int threadsPerBlock = particle_count;
		//float * raw_graph_ptr = thrust::raw_pointer_cast(graphGPU.graphData.data());
		DPSO::Particle * parts = thrust::raw_pointer_cast(gpuParticles.data());
		Moving_Kernel << < 1, threadsPerBlock >> >(parts, bestParticleNum);
		//cudaDeviceSynchronize();
		int new_BestParticleNum = int(thrust::min_element(gpuParticles.begin(), gpuParticles.end()) - gpuParticles.begin());
		Particle tPar = gpuParticles[new_BestParticleNum];
		//std::cout << " - P[" << bestParticleNum << "] = " << best_value << std::endl;
		if (this->best_value > tPar.bestValue)
		{
			this->best_value = tPar.bestValue;
			this->bestParticleNum = new_BestParticleNum;

			best_changed = true;
		}
		return best_changed;
	}

	void SwarmCuda::solve()
	{
		int moves_since_best_changed = 0;
		int64 startTime = cv::getCPUTickCount();
		int iterations = 0;
		int stoppingCount = 20;
		while (moves_since_best_changed <= stoppingCount){
			bool best_changed = false;
			iterations++;
			if (moves_since_best_changed < stoppingCount){  // if cost changing steadily
				best_changed = move_particle();
			}

			if (!best_changed){
				moves_since_best_changed++;
			}
			else{
				moves_since_best_changed = 0;
			}
			std::cout << "Best value so far: " << best_value << std::endl;
		}

		int64 stopTime = cv::getCPUTickCount();
		double timeR = (stopTime - startTime) / cv::getTickFrequency();
		std::cout << "GPU Runtime = " << timeR << "\n Average = " << timeR/iterations;
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

	void SwarmCuda::showParticleData()
	{
		int pSize = graphGPU.num_edgesPerNode;
		//int gSize = graphGPU.num_edgesPerNode*graphGPU.num_nodes;

		for (int i = 0; i < particle_count; i++)
		{
			Particle _p = gpuParticles[i];
			thrust::device_ptr<int> dev_ptr(_p.positionData);
			thrust::device_vector<int> dev_vec(dev_ptr, dev_ptr + pSize);

			std::cout << "Particle : " << i << std::endl;
			std::cout << " - Value : " << _p.psoResult << std::endl;
			std::cout << " - Best : " << _p.bestValue << std::endl;
			std::cout << " - Nodes: ";
			for (int j = 0; j < pSize; j++){	std::cout << dev_vec[j] << " ";	}
			std::cout << std::endl;

			/*thrust::device_ptr<float> g_ptr(_p.graphData);
			thrust::device_vector<float> g_vec(g_ptr, g_ptr + gSize);
			std::cout << " - Graph: \n";
			for (int j = 0; j < pSize; j++)
			{
				for (int k = 0; k < pSize; k++){std::cout << g_vec[j*pSize+k] << " ";}
				std::cout << std::endl;
			}
			std::cout << std::endl;*/

			thrust::device_ptr<int> ptr2(_p.bestPosition);
			thrust::device_vector<int> vec2(ptr2, ptr2 + pSize);
			std::cout << " - BestPosition: ";
			for (int j = 0; j < pSize; j++){	std::cout << vec2[j] << " ";	}
			std::cout << std::endl;

			/*thrust::device_ptr<DPSO::D_Vec2i> ptr3(_p.velocity);
			thrust::device_vector<DPSO::D_Vec2i> vec3(ptr3, ptr3 + pSize);
			std::cout << " - VelocitySize : " << _p.velocitySize << std::endl;
			std::cout << " - Velocity: ";
			for (int j = 0; j < pSize; j++)
			{
				DPSO::D_Vec2i v = vec3[j];
				std::cout << v.from << "-" << v.to << " ";
			}
			std::cout << std::endl;*/
		}
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