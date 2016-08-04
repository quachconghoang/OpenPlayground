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

#include "..\Lib_D-PSO\Swarm.h"

using namespace std;
using namespace ASTAR;
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

void saveNodeData(std::vector<Node> & nodes, std::string fileName)
{
	std::ofstream myfile;
	myfile.open(fileName, std::ofstream::out);
	myfile << nodes.size() << "\n";
	for (int i = 0; i < nodes.size(); i++)
	{
		myfile << i << "\t" << nodes[i].x << "\t" << nodes[i].y << "\t" << nodes[i].z << "\n";
		for (int j = 0; j < nodes[i].cost_to.size(); j++){
			myfile << nodes[i].cost_to[j] << "\t";
		}
		myfile << "\n";
	}

	myfile.close();
}

void astarPathFinding()
{
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

	for (int i = 0; i < captureCloud->size(); i++)
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
	std::cout << "Path finding..." << std::endl;

	std::vector<Point3DInt> waypoints; // vector stores way points

	PathFinder pathFinder(a, b, c);

	std::list<SearchNode> path;
	std::vector<Node> nodes;

	for (int i = 0; i < verticesTrans.size() - 1; i++) {
		PCL_INFO("node %d / %d ... \n", i + 1, verticesTrans.size() - 1);

		Node node;
		node.index = i;
		node.x = verticesFree[i][0];
		node.y = verticesFree[i][1];
		node.z = verticesFree[i][2];

		for (int j = 0; j < verticesTrans.size() - 1; j++)
		{
			path = pathFinder.findPath(world, verticesTrans[i], verticesTrans[j]);

			if (path.size() == 0)
			{
				std::cout << "Path not found..." << std::endl;
				int wpX = verticesTrans[j][0] * gridSize + minPoint.x;
				int wpY = verticesTrans[j][1] * gridSize + minPoint.y;
				int wpZ = verticesTrans[j][2] * gridSize + minPoint.z;
				node.path_to.push_back(Point3DInt(wpX, wpY, wpZ));

				node.cost_to.push_back(DBL_MAX);
			}
			else
			{
				SearchNode *snode = &path.back();
				node.cost_to.push_back(snode->pathCost);

				while (snode->next != nullptr) {
					snode = snode->next;
					int wpX = snode->position[0] * gridSize + minPoint.x;
					int wpY = snode->position[1] * gridSize + minPoint.y;
					int wpZ = snode->position[2] * gridSize + minPoint.z;
					node.path_to.push_back(Point3DInt(wpX, wpY, wpZ));
				}
			}
		}
		nodes.push_back(node);
	}
	//cpfile.close();
	PCL_INFO("End Astar. \n");

	saveNodeData(nodes, "AstarGraph.txt");

	viewer.reset(new PCLVisualizer);
	viewer->setSize(640, 480);
	viewer->addCoordinateSystem(1000.0);
	viewer->addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z, maxPoint.z, 1, 0, 0, "Bound");
	viewer->setShapeRenderingProperties(PCL_VISUALIZER_REPRESENTATION, PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Bound");
	
	viewer->addPointCloud(worldCloud,"world");

	viewer->addPointCloud(captureCloud,"points");
	viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1, 0, 0, "points");
	viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 5, "points");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main()
{
	std::cout << "Running PSO..." << std::endl;
	int particle_count = 128;
	double self_trust = 0; // c1
	double past_trust = 0.5; // parameter for personal best (c2)
	double global_trust = 0.5; // parameter for global best (c3)

	// Init swarm parameters
	Swarm s(particle_count, self_trust, past_trust, global_trust);	
	s.read_graph_definition("..\\..\\IndoorData\\AstarGraph.txt");

	//for (int i = 0; i < particle_count;i++)
	//{
	//	std::cout << "P: " << i << " = " <<s.particles[i].position.to_string() << std::endl;
	//}
	//_getch();
	double val = s.solve();

	std::cout << "Best values: " << s.best_position.to_string() << std::endl;
	std::cout << "Finished!! Press to exit..." << std::endl;

	return 0;
}