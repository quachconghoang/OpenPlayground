// Test_Something.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <conio.h>
#include <iostream>
#include <Eigen/dense>

#include "World.h"
#include "PathFinder.h"
#include "SearchNode.h"

using namespace pcl;
using namespace pcl::visualization;

PCLVisualizer::Ptr viewer;

//=======================
// User Input parameters
double a = 1; // cost weight in X direction
double b = 1; // cost weight in Y direction
double c = 2; // cost weight in Z direction
int gridSize = 50;  // size of grid for path finding (smaller ~ smoother)
int uavRadius = 500;  // size of UAV (its radius)
Eigen::Vector3i p;

int main()
{
	viewer.reset(new PCLVisualizer);
	viewer->setSize(640, 480);
	viewer->addCoordinateSystem(1000.0);

	PointCloud<PointXYZ>::Ptr worldCloud(new PointCloud<PointXYZ>);
	pcl::io::loadPCDFile("..\\..\\IndoorData\\environment.pcd", *worldCloud);
	
	PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*worldCloud, minPoint, maxPoint);

	// Create the environment (UAV world)
	int worldWidth = ceil((maxPoint.x - minPoint.x) / gridSize);
	int worldHeight = ceil((maxPoint.y - minPoint.y) / gridSize);
	int worldDepth = ceil((maxPoint.z - minPoint.z) / gridSize);
	World world(worldWidth, worldHeight, worldDepth); // blank world

	// Import blocked points to the world
	std::cout << "Initialize block points..." << std::endl;
	// Each blocked point must be extended to the UAV size to ensure safe operation of UAV
	int blockRadius = uavRadius / gridSize;
	for (int i = 0; i < worldCloud->points.size(); i++) {
		PointXYZ p = worldCloud->points[i];
		int x = ceil((p.x - minPoint.x) / gridSize);  // Convert to grid coordinate
		int y = ceil((p.y - minPoint.y) / gridSize);
		int z = ceil((p.z - minPoint.z) / gridSize);
		// set a point and its surrounding to be blocked
		world.markPositionCube(Point3DInt(x, y, z), blockRadius, true);
	}

	// Read capture points from file
	PointCloud<PointXYZ>::Ptr captureCloud(new PointCloud<PointXYZ>);
	pcl::io::loadPCDFile("..\\..\\IndoorData\\capturepoint_test.pcd", *captureCloud);
	std::vector<Point3DInt> verticesFree; // non blocking capture points
	std::vector<Point3DInt> verticesTrans;  // non blocking capture points in grid coordinate

	for (int i = 0; i < captureCloud->size();i++)
	{
		PointXYZ p = captureCloud->points[i];
		int x = ceil((p.x - minPoint.x) / gridSize); // convert capture points to grid coordinate
		int y = ceil((p.y - minPoint.y) / gridSize);
		int z = ceil((p.z - minPoint.z) / gridSize);

		if (world.positionIsFree(Point3DInt(x, y, z))) {
			verticesTrans.push_back(Point3DInt(x, y, z));  // non blocking capture points
			verticesFree.push_back(Point3DInt(int(p.x), int(p.y), int(p.z)));  // its value in original coordinate
		}
	}


	///////////////////////////////////////////////////////////////
	// Find path through all non blocking capture points
	// If a point is unreachable, save it to file and then skip it 
	std::cout << "Path finding..." << std::endl;
	//std::ofstream cpfile;
	//cpfile.open("UnreachableCapturePoint.txt");
	std::vector<Point3DInt> waypoints; // vector stores way points
	PathFinder pathFinder(a, b, c);
	std::list<SearchNode> path;
	std::vector<Node> nodes;
	for (int i = 0; i < verticesTrans.size() - 1; i++) {
		PCL_INFO("node %d / %d ... \n", i, verticesTrans.size());

		Node node;
		node.index = i;
		node.x = verticesFree[i].x();
		node.y = verticesFree[i].y();
		node.z = verticesFree[i].z();

		//		std::cout << i << " of " << verticesTrans.size() - 1 << std::endl;
		for (int j = 0; j < verticesTrans.size() - 1; j++) {  // should be refined to be more efficient
			path = pathFinder.findPath(world, verticesTrans[i], verticesTrans[j]);

			if (path.size() == 0) { // No path found
				std::cout << "Path not found..." << std::endl;
				int wpX = verticesTrans[j].x() * gridSize + minPoint.x;
				int wpY = verticesTrans[j].y() * gridSize + minPoint.y;
				int wpZ = verticesTrans[j].z() * gridSize + minPoint.z;
				node.path_to.push_back(Point3DInt(wpX, wpY, wpZ));

				// Add a very large cost value for this path
				node.cost_to.push_back(DBL_MAX);

				// save unreachable point to file for reference
				//cpfile << wpX << " " << wpY << " " << wpZ << std::endl;
			}
			else {  // Path found -> store path to waypoint
				SearchNode *snode = &path.back(); // get the last node ( = start point of path)
				//			std::cout << "=== Cost: " << snode->cost << "- Pathcost:" << snode->pathCost << std::endl;
				node.cost_to.push_back(snode->pathCost);

				while (snode->next != nullptr) {
					snode = snode->next;
					//				std::cout << snode->cost << "- Pathcost:" << snode->pathCost << std::endl;
					int wpX = snode->position.x() * gridSize + minPoint.x;
					int wpY = snode->position.y() * gridSize + minPoint.y;
					int wpZ = snode->position.z() * gridSize + minPoint.z;
					node.path_to.push_back(Point3DInt(wpX, wpY, wpZ));
				}
			}
		}
		nodes.push_back(node);
	}
	//cpfile.close();
	PCL_INFO("End Astar. \n");
	_getch();

	//viewer->addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z, 1, 0, 0, "Bound");
	//viewer->setShapeRenderingProperties(PCL_VISUALIZER_REPRESENTATION, PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Bound");
	////viewer->addPointCloud(worldCloud,"world");

	//viewer->addPointCloud(captureCloud,"points");

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	return 0;
}