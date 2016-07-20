#pragma once

#include <Eigen/dense>

typedef Eigen::Vector3i Point3DInt;

inline double distancePoint3DInt(Point3DInt & a, Point3DInt & b)
{
	return (a - b).cwiseAbs2().sum();
}

class World
{
public:
	World(int width, int height, int depth);

	//Note: we use Y as height and Z as depth here!
	const int getLeft();
	const int getRight();
	const int getBottom();
	const int getTop();
	const int getFront();
	const int getBack();

	void markPosition(Point3DInt position, bool value);
	void markPositionCube(Point3DInt centerPos, int cubeRadius, bool value);
	bool positionIsFree(Point3DInt position);

private:
	int sx;
	int sy;
	int sz;
	int offsetIdx;
	int maxIdx;
	bool *worldBlocked; //extremely simple world where each node can be free or blocked: true=blocked        
	void markPositionEx(Point3DInt position, bool value);

};

