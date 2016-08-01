#ifndef GPU_PARTICLE_H
#define GPU_PARTICLE_H

namespace DPSO
{
	struct Position
	{
		std::vector<int> nodeID;
	};

	struct Vec2i
	{
		int from;
		int to;
	};

	struct Velocity
	{
		std::vector<Vec2i> vec;
	};
}

#endif