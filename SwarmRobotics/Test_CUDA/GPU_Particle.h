#ifndef GPU_PARTICLE_H
#define GPU_PARTICLE_H

#include <vector>
#include "thrust/host_vector.h"
#include "thrust/device_vector.h"
#include "thrust/device_ptr.h"

namespace DPSO
{
	// Create struct object to sorting by THRUST
	struct Particle
	{
		int pIndex;

		int graphOffset;

		int positionOffset;
		int positionSize;
		int * positionData;
		
		
		float self_trust;
		float past_trust;
		float global_trust;

		float bestValue;
		int * bestPosition;
		
		float psoResult;

		__host__ __device__
		bool operator<(const Particle other) const
		{
			return psoResult < other.psoResult;
		}
	};
}

//typedef std::vector<Vec2i> Veloc;
//typedef std::vector<int> Posit;
#endif