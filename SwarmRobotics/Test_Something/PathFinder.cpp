#include "stdafx.h"
#include "PathFinder.h"
#include "MinHeap.h"

namespace ASTAR
{
	PathFinder::PathFinder(double a, double b, double c) {
		// Cost to move from a point to its neighbors
		surrounding.push_back(Surr(-1, 1, 1, a, b, c));
		surrounding.push_back(Surr(0, 1, 1, a, b, c));
		surrounding.push_back(Surr(1, 1, 1, a, b, c));
		surrounding.push_back(Surr(-1, 1, 0, a, b, c));
		surrounding.push_back(Surr(0, 1, 0, a, b, c));
		surrounding.push_back(Surr(1, 1, 0, a, b, c));
		surrounding.push_back(Surr(-1, 1, -1, a, b, c));
		surrounding.push_back(Surr(0, 1, -1, a, b, c));
		surrounding.push_back(Surr(1, 1, -1, a, b, c));
		surrounding.push_back(Surr(-1, 0, 1, a, b, c));
		surrounding.push_back(Surr(0, 0, 1, a, b, c));
		surrounding.push_back(Surr(1, 0, 1, a, b, c));
		surrounding.push_back(Surr(-1, 0, 0, a, b, c));
		surrounding.push_back(Surr(1, 0, 0, a, b, c));
		surrounding.push_back(Surr(-1, 0, -1, a, b, c));
		surrounding.push_back(Surr(0, 0, -1, a, b, c));
		surrounding.push_back(Surr(1, 0, -1, a, b, c));
		surrounding.push_back(Surr(-1, -1, 1, a, b, c));
		surrounding.push_back(Surr(0, -1, 1, a, b, c));
		surrounding.push_back(Surr(1, -1, 1, a, b, c));
		surrounding.push_back(Surr(-1, -1, 0, a, b, c));
		surrounding.push_back(Surr(0, -1, 0, a, b, c));
		surrounding.push_back(Surr(1, -1, 0, a, b, c));
		surrounding.push_back(Surr(-1, -1, -1, a, b, c));
		surrounding.push_back(Surr(0, -1, -1, a, b, c));
		surrounding.push_back(Surr(1, -1, -1, a, b, c));
	}

	// Compute cost with weighted factor
	PathFinder::Surr::Surr(int x, int y, int z, double a, double b, double c)
	{
		Point = Point3DInt(x, y, z);
		Cost = (a*x*x + b*y*y + c*z*z);
	}

	std::list<SearchNode> PathFinder::findPath(World& world, Point3DInt& start, Point3DInt& end)
	{
		// note we just flip start and end here so you don't have to.            
		return findPathReversed(world, end, start);
	}

	// Find path using A* 
	std::list<SearchNode> PathFinder::findPathReversed(World& world, Point3DInt& start, Point3DInt& end)
	{
		std::list<SearchNode> closedList; // List stores the found path
		SearchNode startNode = SearchNode(&start, 0, 0, nullptr);

		MinHeap openList = MinHeap();  // List stores nodes to be explored
		openList.insert(startNode);

		int sx = world.getRight();
		int sy = world.getTop();
		int sz = world.getBack();
		std::vector<bool> brWorld(sx*sy*sz, false);
		brWorld[start[0] + (start[1] + start[2] * sy) * sx] = true;
		int i = 1;
		while (openList.size())	{
			i++;
			closedList.push_back(openList.extractMin());  // get the node with minimum cost
			SearchNode *current = &closedList.back();
			openList.deleteMin();

			// path found
			if (distancePoint3DInt(current->position, end) == 0)	{
				closedList.push_back(SearchNode(&end, current->cost, current->pathCost, current));
				return closedList;
			}

			// For each unblocked neighbor, compute the cost to it
			// then add to the open list
			for (int i = 0; i < surrounding.size(); i++) {
				Surr surr = surrounding[i];
				Point3DInt neighborPoint = current->position + surr.Point;
				int brWorldIdx = neighborPoint[0] + (neighborPoint[1] + neighborPoint[2] * sy) * sx;

				if (world.positionIsFree(neighborPoint) && brWorld[brWorldIdx] == false) {
					brWorld[brWorldIdx] = true;
					double pathCost = current->pathCost + surr.Cost;
					double cost = pathCost + distancePoint3DInt(neighborPoint, end);
					SearchNode node = SearchNode(&Point3DInt(neighborPoint[0], neighborPoint[1], neighborPoint[2]), cost, pathCost, current);
					openList.insert(node);
				}
			}
		}
		std::list<SearchNode> emptList;
		return emptList; // No path found
	}
}