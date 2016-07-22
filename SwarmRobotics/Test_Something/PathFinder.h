#pragma once

#include "SearchNode.h"
#include "World.h"
#include <vector>
#include <list>
#include <memory>


namespace ASTAR
{
	class PathFinder
	{
	private:
		class Surr
		{
		public:
			Surr(int x, int y, int z, double a, double b, double c);
			Point3DInt Point;
			double Cost;
		};

	public:
		PathFinder(double a, double b, double c);
		std::list<SearchNode> findPath(World& world, Point3DInt& start, Point3DInt& end);

	private:
		std::list<SearchNode> findPathReversed(World& world, Point3DInt& start, Point3DInt& end);
		std::vector<Surr> surrounding;
	};
}
