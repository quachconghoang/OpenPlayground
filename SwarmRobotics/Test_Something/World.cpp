//*************************************//
// Creates a 3D world
//*************************************//

#include "stdafx.h"
#include "World.h"

World::World(int width, int height, int depth) {

	// add 2 to compensate for the solid border around the world
	sx = width + 2;
	sy = height + 2;
	sz = depth + 2;
	offsetIdx = (0 + 1) + ((0 + 1) + (0 + 1) * sy) * sx;
	maxIdx = sx * sy * sz;
	worldBlocked = new bool[sx * sy * sz] ();

	// add solid border
	for (int x = 0; x < sx; ++x)
	{
		for (int y = 0; y < sy; ++y)
		{
			markPositionEx(Point3DInt(x, y, 0), true);
			markPositionEx(Point3DInt(x, y, sz - 1), true);
		}
	}

	for (int y = 0; y < sy; ++y)
	{
		for (int z = 0; z < sz; ++z)
		{
			markPositionEx(Point3DInt(0, y, z), true);
			markPositionEx(Point3DInt(sx - 1, y, z), true);
		}
	}

	for (int z = 0; z < sz; ++z)
	{
		for (int x = 0; x < sx; ++x)
		{
			markPositionEx(Point3DInt(x, 0, z), true);
			markPositionEx(Point3DInt(x, sy - 1, z), true);
		}
	}
}

// Mark a position as blocked or free
void World::markPosition(Point3DInt position, bool value)
{
	int idx = offsetIdx + position[0] + (position[1] + position[2] * sy) * sx;
	if (idx > offsetIdx && idx < maxIdx)	worldBlocked[idx] = value;
}

// Mark a position and its surrounding as blocked or free
void World::markPositionCube(Point3DInt centerPos, int cubeRadius, bool value)
{
	int yInc, zInc;
	int centerIndex = offsetIdx + centerPos[0] + (centerPos[1] + centerPos[2] * sy) * sx;
	for (int i = -cubeRadius; i <= cubeRadius; i++) 
		if ((centerPos[2] + i) >= getFront() && (centerPos[2] + i) <= getBack()) {
			zInc = i*sy*sx;
			for (int j = -cubeRadius; j <= cubeRadius; j++)
				if ((centerPos[1] + j) >= getBottom() && (centerPos[1] + j) <= getTop()) {
					yInc = j*sx; 
					for (int k = -cubeRadius; k <= cubeRadius; k++)
						if ((centerPos[0] + k) >= getLeft() && (centerPos[0] + k) <= getRight())
							worldBlocked[centerIndex + k + yInc + zInc] = value;
				}
		}
}

void World::markPositionEx(Point3DInt position, bool value)
{
	worldBlocked[position[0] + (position[1] + position[2] * sy) * sx] = value;
}

bool World::positionIsFree(Point3DInt position)
{
	return !worldBlocked[offsetIdx + position[0] + (position[1] + position[2] * sy) * sx];
}

const int World::getLeft() {
	return 0;
}

const int World::getRight() {
	return sx - 2;
}

const int World::getBottom() {
	return 0;
}

const int World::getTop() {
	return sy - 2;
}

const int World::getFront() {
	return 0;
}

const int World::getBack() {
	return sz - 2;
}