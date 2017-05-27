#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include <queue>
#include <vector>
#include <functional>
#include <list>

using namespace std;

#define INF 1000000
#define MAP_SIZE 10
#define NUM_ROBOT 4
#define NUM_TASK 16
#define MAX_ENERGY 2000
#define TIME_MAX 400

#define IDLE 0
#define WORKING 1
#define MOVING 2

#define UP 'w'
#define DOWN 's'
#define LEFT 'a'
#define RIGHT 'd'

/* 
First global variables definition
*/
typedef pair<int, int> iPair;

int hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
int terreinMatrix[MAP_SIZE][MAP_SIZE];

int hWallSearch[MAP_SIZE - 1][MAP_SIZE] = { 0, };
int vWallSearch[MAP_SIZE][MAP_SIZE - 1] = { 0, };

/*
Class : Coordinate
*/
class Coordinate
{
public:
	Coordinate();
	Coordinate(int xx, int yy);

	int x;
	int y;
};

Coordinate::Coordinate()
{
	x = 0;
	y = 0;
}

Coordinate::Coordinate(int xx, int yy)
{
	x = xx;
	y = yy;
}

/*
Class : Node
*/
class Node
{
public:
	Node(int xx, int yy, int cost);
	Node();

	int nodeNum;
	unsigned int nodeCost;	// cost
	Node *prev = NULL;
	Coordinate coord;
};

Node::Node() {
	nodeCost = INF;
	nodeNum = -1;
}

Node::Node(int xx, int yy, int cost) {
	nodeCost = cost;
	nodeNum = MAP_SIZE * xx + yy;
	Coordinate coord = Coordinate(xx, yy);
}

/*
Class : Task
*/
class Task
{
public:
	Coordinate taskCoord;
	std::vector<Coordinate> path;
	int taskId;
	int taskCost;
	int taskStatus;
};

/*
Class : Robot
*/
class Robot
{
public:
	Robot();

	class Graph
	{
	public:
		int v;    // No. of vertices
				  // In a weighted graph, we need to store vertex
				  // and weight pair for every edge
		list< pair<int, int> > *adj;

		Graph();
		Graph(int V);  // Constructor
					   // function to add an edge to graph
		void addEdge(int u, int v, int w1, int w2);
		void deleteEdge(int u, int v, int w);
		// prints shortest path from s
		void shortestPath(int s, Robot *robot);
	};

	int robotNum;
	
	Coordinate robotCoord;
	Coordinate taskCoord[NUM_TASK];
	int travelCost[MAP_SIZE][MAP_SIZE]; //travel cost of block Coordinate
	int taskCost[NUM_TASK]; // cost of a task performed by this robot
	int distance[MAP_SIZE * MAP_SIZE];
	Task taskList[NUM_TASK];// list of tasks assgined to this robot

	Node nodeMap[MAP_SIZE * MAP_SIZE];
	int itemCost[NUM_TASK];
	vector<Coordinate> itemPath[NUM_TASK];

	void setRobotNum(int i);

	void setNodeMap();

	Graph robotGraph;
	void setCostEdge();
	void setItemPath(Task taskList[]);
	void setItemCost(Coordinate itemCoord[]);

	int totalCost;// total energy consumed
	int totalBlocks; // total number of blocks traveled

	int numTask;
	int status;
	int energy;

	int currTask;
	int pathIndex;

	//void assignTask(int taskId, std::vector<Coordinate> inputPath, Coordinate itemList[]);
	void assignTask(Task task, std::vector<Coordinate> inputPath);

	void calcCost();

	int getTravelCost();
	int getTravelCost(int x, int y);

	int getTaskCost();
	int getTaskCost(int x);

	Coordinate getCurrentPosition();

	void updatePostion();
	bool recognizeWalls();
	bool atTask();
};

Robot::Robot() {
	robotCoord.x = 0;
	robotCoord.y = 0;

	Graph robotGraph = Graph(MAP_SIZE * MAP_SIZE);

	numTask = 0;

	energy = MAX_ENERGY;

	status = IDLE;
	currTask = 0;
	pathIndex = 0;

	for (int ii = 0; ii < NUM_TASK; ii++)
	{
		taskCoord[ii].x = 0;
		taskCoord[ii].y = 0;
		taskCost[ii] = 0;

		taskList[ii].taskId = -1;
	}

	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			travelCost[ii][jj] = 0;
		}
	}
	totalCost = 0;
	totalBlocks = 0;
}

/*
void Robot::assignTask(int taskId, std::vector<Coordinate> inputPath, Coordinate itemList[])
{
	Coordinate current;
	Coordinate next;

	int pass = 0;

	if (numTask < NUM_TASK)
	{
		if (numTask == 0)
		{
			current.x = robotCoord.x;
			current.y = robotCoord.y;
		}
		else
		{
			current.x = itemList[taskList[numTask - 1].taskId].x;
			current.y = itemList[taskList[numTask - 1].taskId].y;
		}

		next.x = itemList[taskId].x;
		next.y = itemList[taskId].y;

		printf("Robot %d path assigned from (%d, %d) to (%d, %d)\n", robotNum, current.x, current.y, next.x, next.y);

		//pass = pathValidation(inputPath, current, next);

		if (pass == 0)
		{
			taskList[numTask].taskId = taskId;
			taskList[numTask].path = inputPath;
			taskList[numTask].taskCoord.x = next.x;
			taskList[numTask].taskCoord.y = next.y;
			numTask++;
		}
		else
		{
			printf("assignment validation failed\n");
		}
	}
	else
	{
		printf("too many task assignment failed\n");
	}
}*/

void Robot::assignTask(Task task, std::vector<Coordinate> inputPath)
{
	Coordinate current;
	Coordinate next;

	int pass = 0;

	if (numTask < NUM_TASK)
	{

		current.x = robotCoord.x;
		current.y = robotCoord.y;
		
		next.x = task.taskCoord.x;
		next.y = task.taskCoord.y;

		printf("Robot %d path assigned from (%d, %d) to (%d, %d)\n", robotNum, current.x, current.y, next.x, next.y);

		//pass = pathValidation(inputPath, current, next);

		if (pass == 0)
		{
			taskList[numTask].taskId = task.taskId;
			taskList[numTask].path = inputPath;
			taskList[numTask].taskCoord.x = next.x;
			taskList[numTask].taskCoord.y = next.y;
			numTask++;
		}
		else
		{
			printf("assignment validation failed\n");
		}
	}
	else
	{
		printf("too many task assignment failed\n");
	}
}

void Robot::calcCost()
{
	int tempCost = 0;
	int tempBlocks = 0;
	int costList = 0;

	for (int ii = 0; ii < numTask; ii++)
	{
		tempCost = 0;
		tempBlocks = 0;

		if (taskList[ii].path.empty() == false)
		{
			for (int jj = 0; jj < taskList[ii].path.size(); jj++)
			{
				tempCost += travelCost[taskList[ii].path.at(jj).y][taskList[ii].path.at(jj).x];
				tempBlocks++;
			}
		}

		totalCost += tempCost;
		totalBlocks += tempBlocks;

		totalCost += taskCost[taskList[ii].taskId];
	}
}

int Robot::getTravelCost()
{
	return travelCost[robotCoord.y][robotCoord.x];
}

int Robot::getTravelCost(int x, int y)
{
	return travelCost[y][x];
}

int Robot::getTaskCost()
{
	return taskCost[taskList[currTask].taskId];
}

int Robot::getTaskCost(int x)
{
	return taskCost[x];
}

Coordinate Robot::getCurrentPosition()
{
	return robotCoord;
}

bool Robot::atTask()
{
	if (robotCoord.x == taskList[currTask].taskCoord.x && robotCoord.y == taskList[currTask].taskCoord.y)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Robot::setRobotNum(int i)
{
	robotNum = i;
}

void Robot::setNodeMap()
{
	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			nodeMap[MAP_SIZE * ii + jj].nodeNum = MAP_SIZE * ii + jj;
			nodeMap[MAP_SIZE * ii + jj].nodeCost = travelCost[ii][jj];
			nodeMap[MAP_SIZE * ii + jj].coord.x = jj;
			nodeMap[MAP_SIZE * ii + jj].coord.y = ii;
		}
	}
}

/*
Function : recognizeWalls
recognize walls around the robot
*/
bool Robot::recognizeWalls()
{
	bool newWall = false;
	int x = robotCoord.x;
	int y = robotCoord.y;

	/* vertical walls */
	if (x != 0) {
		if (vWallMatrix[y][x - 1] == 1) {
			if (vWallSearch[y][x - 1] != 1) {
				vWallSearch[y][x - 1] = 1;
				printf("(%d,%d) vwall recognized\n", x - 1, y);
				newWall = true;
			}
		}
	}

	if (x != MAP_SIZE - 1) {
		if (vWallMatrix[y][x] == 1) {
			if (vWallSearch[y][x] != 1) {
				vWallSearch[y][x] = 1;
				printf("(%d,%d) vwall recognized\n", x, y);
				newWall = true;
			}
		}
	}

	/* horizontal walls */
	if (y != 0) {
		if (hWallMatrix[y - 1][x] == 1) {
			if (hWallSearch[y - 1][x] != 1) {
				hWallSearch[y - 1][x] = 1;
				printf("(%d,%d) hwall recognized\n", x, y - 1);
				newWall = true;
			}
		}
	}

	if (y != MAP_SIZE - 1) {
		if (hWallMatrix[y][x] == 1) {
			if (hWallSearch[y][x] != 1) {
				hWallSearch[y][x] = 1;
				printf("(%d,%d) hwall recognized\n", x, y);
				newWall = true;
			}
		}
	}

	return newWall;
}

void Robot::setCostEdge()
{
	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			if (ii == 0) {
				if (jj == 0) {
					if (vWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					}
					if (hWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);	//down
					}
				}
				else if (jj == 9) {
					if (vWallSearch[ii][jj - 1] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					}
					if (hWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					}
				}
				else {
					if (vWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					}
					if (vWallSearch[ii][jj - 1] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					}
					if (hWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					}
				}
			}
			else if (ii == 9)
			{
				if (jj == 0)
				{
					if (hWallSearch[ii - 1][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					}
					if (vWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					}
				}
				else if (jj == 9)
				{
					if (hWallSearch[ii - 1][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					}
					if (vWallSearch[ii][jj - 1] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					}
				}
				else
				{
					if (hWallSearch[ii - 1][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					}
					if (vWallSearch[ii][jj - 1] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					}
					if (vWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					}
				}
			}
			else
			{
				if (jj == 0) {
					if (hWallSearch[ii - 1][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					}
					if (vWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					}
					if (hWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					}
				}
				else if (jj == 9) {
					if (hWallSearch[ii - 1][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					}
					if (vWallSearch[ii][jj - 1] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					}
					if (hWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					}
				}
				else {
					if (hWallSearch[ii - 1][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					}
					if (vWallSearch[ii][jj - 1] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					}
					if (vWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					}
					if (hWallSearch[ii][jj] == 0) {
						robotGraph.addEdge(MAP_SIZE * ii + jj, MAP_SIZE * (ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					}
				}
			}
		}
	}
}

void Robot::setItemPath(Task taskList[])
{
	Coordinate tempCoord;
	for (int i = 0; i < NUM_TASK; i++)
	{
		// printf("Itemcoord input %d %d \n", Itemcoord[i].x, Itemcoord[i].y);
		if (taskList[i].taskCoord.x != 11) {
			itemPath[i].push_back(taskList[i].taskCoord);
			while (nodeMap[MAP_SIZE * taskList[i].taskCoord.y + taskList[i].taskCoord.x].prev != NULL)
			{
				tempCoord.x = nodeMap[MAP_SIZE * taskList[i].taskCoord.y + taskList[i].taskCoord.x].coord.x;
				tempCoord.y = nodeMap[MAP_SIZE * taskList[i].taskCoord.y + taskList[i].taskCoord.x].coord.y;
				itemPath[i].push_back(tempCoord);
				nodeMap[MAP_SIZE * taskList[i].taskCoord.y + taskList[i].taskCoord.x] = * nodeMap[MAP_SIZE * taskList[i].taskCoord.y + taskList[i].taskCoord.x].prev;
				//printf("Node <%d %d> \n", tempCoord.y, tempCoord.x);
			}
		}
	}
}

void Robot::setItemCost(Coordinate itemCoord[])
{
	Coordinate tempCoord;
	for (int i = 0; i < NUM_TASK; i++)
	{
		tempCoord = itemCoord[i];
		int cost = taskCost[i] + distance[MAP_SIZE * itemCoord[i].y + itemCoord[i].x];
		itemCost[i] = cost;
	}
}

void Robot::updatePostion()
{
	pathIndex++;

	if (pathIndex < taskList[currTask].path.size())
	{
		robotCoord.x = taskList[currTask].path.at(pathIndex).x;
		robotCoord.y = taskList[currTask].path.at(pathIndex).y;
	}
	else
	{
		pathIndex = 0;
	}
}

// Allocates memory for adjacency list
Robot::Graph::Graph()
{
	this->v = MAP_SIZE * MAP_SIZE;
	adj = new list<iPair>[v];
}

Robot::Graph::Graph(int v)
{
	this->v = v;
	adj = new list<iPair>[v];
}

void Robot::Graph::addEdge(int u, int v, int w1, int w2)
{
	adj[u].push_back(make_pair(v, w1));
}

void Robot::Graph::deleteEdge(int u, int v, int w)
{
	adj[u].remove(make_pair(v, w));
}

// Prints shortest paths from src to all other vertices
void Robot::Graph::shortestPath(int src, Robot *robot)
{
	priority_queue< iPair, vector <iPair>, greater<iPair> > pq;

	// Create a vector for distances and initialize all
	// distances as infinite (INF)
	vector<int> dist(v, INF);

	// Insert source itself in priority queue and initialize
	// its distance as 0.
	pq.push(make_pair(0, src));
	dist[src] = 0;

	/* Looping till priority queue becomes empty (or all
	distances are not finalized) */
	while (!pq.empty())
	{
		// The first vertex in pair is the minimum distance
		// vertex, extract it from priority queue.
		// vertex label is stored in second of pair (it
		// has to be done this way to keep the vertices
		// sorted distance (distance must be first item
		// in pair)
		// u = from, v = to
		int u = pq.top().second;
		pq.pop();

		// 'i' is used to get all adjacent vertices of a vertex
		list< pair<int, int> >::iterator i;
		for (i = adj[u].begin(); i != adj[u].end(); ++i)
		{
			// Get vertex label and weight of current adjacent
			// of u.
			int v = (*i).first;
			int weight = (*i).second;

			//  If there is shorted path to v through u.
			if (dist[v] > dist[u] + weight)
			{
				// Updating distance of v
				dist[v] = dist[u] + weight;
				(*robot).nodeMap[v].prev = &(*robot).nodeMap[u];
				//printf("%d %d\n", (*robot).Nodemap[v].nodeNum, (*robot).Nodemap[v].prev->nodeNum);
				pq.push(make_pair(dist[v], v));
			}
		}
	}
	// Print shortest distances stored in dist[]
	//printf("Vertex   Distance from Source\n");
	for (int i = 0; i < v; ++i){
		//printf("%d \t\t %d\n", i, dist[i]);
		(*robot).distance[i] = dist[i];
	}
	Node copymap[MAP_SIZE * MAP_SIZE];
	memcpy(&copymap, &(*robot).nodeMap, sizeof((*robot).nodeMap));
	//Node point = *copymap;
	/*for (int i = 0; i < v; i++)
	{
		Node point = copymap[i];
		printf("%d : ", copymap[i].nodeNum);
		while (point.prev != NULL) {
			printf("%d ", point.prev->nodeNum);
			point = *point.prev;
		}
		printf("\n");
	}*/
	
}

/*
Class : RobotToTask
*/
class RobotToTask
{
public:
	int robotID;
	int taskID;
	std::vector <Coordinate> path;
	int cost;
};

/*
Class : RobotToTasks
*/
class RobotToTasks
{
public:
	int robotID;
	RobotToTask * list;
};

int pathValidation(std::vector <Coordinate> inputPath, Coordinate start, Coordinate end);
void print_result(Robot * robotList, int mode);
std::vector <Coordinate> pathGeneration(Coordinate start, Coordinate end);

void print_result(Robot  * robotList, int mode)
{
	int totalConsumption = 0;

	printf("results\n");

	totalConsumption = 0;

	for (int ii = 0; ii < NUM_ROBOT; ii++)
	{
		printf("robot %d energy : %d\n", ii, robotList[ii].totalCost);
		totalConsumption += robotList[ii].totalCost;
	}
	printf("energy consumption total : %d\n", totalConsumption);

	for (int ii = 0; ii < NUM_ROBOT; ii++)
	{
		printf("robot %d path length : %d\n", ii, robotList[ii].totalBlocks);
	}
	printf("\n");
}

int pathValidation(std::vector <Coordinate> inputPath, Coordinate start, Coordinate end)
{

	for (int ii = 0; ii < inputPath.size() - 1; ii++) {
		int pathx = inputPath.at(ii).x;
		int pathy = inputPath.at(ii).y;
		//printf("path (%d,%d)\n", pathx, pathy);
	}
	int pass = 0;

	if (inputPath.at(0).x != start.x || inputPath.at(0).y != start.y)
	{
		pass = 1;
	}
	if (inputPath.at(inputPath.size() - 1).x != end.x || inputPath.at(inputPath.size() - 1).y != end.y)
	{
		pass = 2;
	}

	int xdiff = 0;
	int ydiff = 0;

	int prevx = 0;
	int prevy = 0;
	int nextx = 0;
	int nexty = 0;

	for (int ii = 0; ii < inputPath.size() - 1; ii++)
	{
		nextx = inputPath.at(ii + 1).x;
		nexty = inputPath.at(ii + 1).y;
		prevx = inputPath.at(ii).x;
		prevy = inputPath.at(ii).y;

		xdiff = nextx - prevx;
		ydiff = nexty - prevy;

		if (xdiff == -1 && ydiff == 0)
		{
			//left
			if (vWallMatrix[prevy][prevx - 1] == 1)
			{
				pass = 4;
			}
		}
		else if (xdiff == 1 && ydiff == 0)
		{
			//right
			if (vWallMatrix[prevy][prevx] == 1)
			{
				pass = 4;
			}
		}
		else if (ydiff == -1 && xdiff == 0)
		{
			//up
			if (hWallMatrix[prevy - 1][prevx] == 1)
			{
				pass = 4;
			}
		}
		else if (ydiff == 1 && xdiff == 0)
		{
			//down
			if (hWallMatrix[prevy][prevx] == 1)
			{
				pass = 4;
			}
		}
		else
		{
			pass = 8;
		}
	}
	if (pass > 0)
	{
		printf("invalid path : ");

		if (pass >= 8)
		{
			printf("jump path,");
			pass -= 8;
		}

		if (pass >= 4)
		{
			printf("path through wall,");
			pass -= 4;
		}
		if (pass >= 2)
		{
			printf("end position,");
			pass -= 2;
		}
		if (pass == 1)
		{
			printf("start position");
			pass -= 1;
		}
		printf("\n");
		pass = 1;
	}
	return pass;
}

int pathValidation(std::vector <Coordinate> inputPath)
{

	if (inputPath.size() == 0) {
		return 0;
	}
	for (int ii = 0; ii < inputPath.size() - 1; ii++) {
		int pathx = inputPath.at(ii).x;
		int pathy = inputPath.at(ii).y;
		//printf("path (%d,%d)\n", pathx, pathy);
	}
	int pass = 0;

	int xdiff = 0;
	int ydiff = 0;

	int prevx = 0;
	int prevy = 0;
	int nextx = 0;
	int nexty = 0;

	for (int ii = 0; ii < inputPath.size() - 1; ii++)
	{
		nextx = inputPath.at(ii + 1).x;
		nexty = inputPath.at(ii + 1).y;
		prevx = inputPath.at(ii).x;
		prevy = inputPath.at(ii).y;

		xdiff = nextx - prevx;
		ydiff = nexty - prevy;

		if (xdiff == -1 && ydiff == 0)
		{
			//left
			if (vWallMatrix[prevy][prevx - 1] == 1)
			{
				pass = 4;
			}
		}
		else if (xdiff == 1 && ydiff == 0)
		{
			//right
			if (vWallMatrix[prevy][prevx] == 1)
			{
				pass = 4;
			}
		}
		else if (ydiff == -1 && xdiff == 0)
		{
			//up
			if (hWallMatrix[prevy - 1][prevx] == 1)
			{
				pass = 4;
			}
		}
		else if (ydiff == 1 && xdiff == 0)
		{
			//down
			if (hWallMatrix[prevy][prevx] == 1)
			{
				pass = 4;
			}
		}
		else
		{
			pass = 8;
		}
	}
	if (pass > 0)
	{
		printf("invalid path : ");

		if (pass >= 8)
		{
			printf("jump path,");
			pass -= 8;
		}

		if (pass >= 4)
		{
			printf("path through wall,");
			pass -= 4;
		}
		if (pass >= 2)
		{
			printf("end position,");
			pass -= 2;
		}
		if (pass == 1)
		{
			printf("start position");
			pass -= 1;
		}
		printf("\n");
		pass = 1;
	}
	return pass;
}

vector <Coordinate> pathRev(vector <Coordinate> tempPath) {
	vector <Coordinate> path;
	vector <Coordinate> pathRev = tempPath;

	for (int i = 0; i < tempPath.size(); i++) {
		path.push_back(tempPath.back());
		tempPath.pop_back();
	}
	return path;
}

/*
Function : pathGeneration
temporary function to generate path
*/
std::vector <Coordinate> pathGeneration(Coordinate start, Coordinate end) {
	std::vector <Coordinate> path;

	Coordinate currentPos;
	Coordinate itemPos = end;

	currentPos.x = start.x;
	currentPos.y = start.y;

	path.push_back(currentPos);

	int xDiff = itemPos.x - currentPos.x;
	int yDiff = itemPos.y - currentPos.y;

	while (xDiff != 0 || yDiff != 0)
	{
		if (xDiff > 0) {
			currentPos.x = currentPos.x + 1;
		}
		else if (xDiff == 0) {
			currentPos.x = currentPos.x;
			if (yDiff > 0) {
				currentPos.y = currentPos.y + 1;
			}
			else if (yDiff == 0) {
				currentPos.y = currentPos.y;
			}
			else {
				currentPos.y = currentPos.y - 1;
			}
		}
		else {
			currentPos.x = currentPos.x - 1;
		}

		path.push_back(currentPos);

		xDiff = itemPos.x - currentPos.x;
		yDiff = itemPos.y - currentPos.y;
	}
	return path;
}

/*
Second global variables definition
*/
int doneList[NUM_TASK] = { 0, };

Task totalTaskList[NUM_TASK];
Robot robotList[NUM_ROBOT];


/*
Function : scheduling
perform min-min scheduling (need to revise later)
*/
void scheduling() {
	int targetTasks[NUM_TASK] = { 0, };
	int scheduleTable[NUM_TASK][NUM_ROBOT];
	int robotNeedToSchedule = 0;
	int taskNeedToSchedule = 0;
	for (int j = 0; j < NUM_TASK; j++) {
		for (int i = 0; i < NUM_ROBOT; i++) {
			scheduleTable[j][i] = INF;
		}
	}
	for (int i = 0; i < NUM_TASK; i++) {
		if (totalTaskList[i].taskStatus == MOVING) {
			taskNeedToSchedule++;
			for (int j = 0; j < NUM_ROBOT; j++) {
				if (robotList[j].status == IDLE) {
					scheduleTable[i][j] = robotList[j].distance[MAP_SIZE * totalTaskList[i].taskCoord.y + totalTaskList[i].taskCoord.x];
				}
			}
			targetTasks[i] = 1;
		}
	}

	for (int j = 0; j < NUM_ROBOT; j++) {
		if (robotList[j].status == IDLE) {
			robotNeedToSchedule++;
		}
	}

	if (robotNeedToSchedule == 0 || taskNeedToSchedule == 0) {
		printf("Scheduling is not needed\n");
		return;
	}

	int scheduledRobot[NUM_ROBOT] = {-1,-1,-1,-1};
	int scheduledTask[NUM_TASK] = { -1,-1,-1,-1,-1,-1,-1,-1 ,-1,-1,-1,-1 ,-1,-1,-1,-1 };

	int minCostPerTask[NUM_TASK] = {INF,INF,INF,INF, INF, INF, INF, INF, INF,INF,INF,INF, INF, INF, INF, INF};
	int robotIDs[NUM_TASK] = { -1,-1,-1,-1,-1,-1,-1,-1 ,-1,-1,-1,-1 ,-1,-1,-1,-1 };

	// find min cost per task
	for (int j = 0; j < NUM_TASK; j++) {
		int robotID = -1;
		int minCost = INF;

		for (int i = 0; i < NUM_ROBOT; i++)
		{
			if (scheduleTable[j][i] < minCost) {
				robotID = i;
				minCost = scheduleTable[j][i];
			}
		}
		robotIDs[j] = robotID;
		minCostPerTask[j] = minCost;
	}

	// scheduling
	for (int i = 0; i < robotNeedToSchedule; i++) {
		/*for (int j = 0; j < NUM_TASK; j++) {
			printf("%d cost task %d\n", minCostPerTask[j], j);
		}*/

		int taskID = -1;
		int minCost = INF;

		// find min cost task and robot
		for (int j = 0; j < NUM_TASK; j++) {
			if (scheduledTask[j] == -1) {
				if (minCostPerTask[j] < minCost) {
					taskID = j;
					minCost = minCostPerTask[j];
					//printf("min is %d, %d\n", taskID, minCost);
				}
			}
		}

		if (taskID == -1) {
			printf("There is no task to schedule\n");
			break;
		}

		int selectedRobot = robotIDs[taskID];
		scheduledRobot[selectedRobot] = taskID;
		scheduledTask[taskID] = selectedRobot;
		printf("robot%d is allocated to task%d\n", selectedRobot, taskID);
		minCostPerTask[taskID] = INF;
		totalTaskList[taskID].taskStatus = WORKING;
		robotList[selectedRobot].assignTask(totalTaskList[taskID], robotList[selectedRobot].itemPath[taskID]);

		// update minCostPerTask
		for (int j = 0; j < NUM_TASK; j++) {
			if (scheduledTask[j] == -1) {
				if (selectedRobot == robotIDs[j]) {
					int robotID = -1;
					int minCost = INF;

					for (int i = 0; i < NUM_ROBOT; i++)
					{
						if (scheduleTable[j][i] < minCost && scheduledRobot[i] == -1) {
							robotID = i;
							minCost = scheduleTable[j][i];
						}
					}
					robotIDs[j] = robotID;
					minCostPerTask[j] = minCost;
				}
			}
			else {
				continue;
			}
		}
	}

	for (int i = 0; i < NUM_ROBOT; i++)
	{
		//printf("robot %d is assigned to task %d with total cost %d \n", i, scheduledRobot[i], robotList[i].distance[MAP_SIZE * robotList[i].taskCoord[scheduledRobot[i]].y + robotList[i].taskCoord[scheduledRobot[i]].x]);
	}
}

/*
Function : buildMap
initialization of maps
*/
void buildMap() {
	char data;

	/* true map loading and setting */
	FILE  * fp;
	fopen_s(&fp, "hwall10.txt", "r");

	int i = 0;
	int j = 0;

	while (fscanf_s(fp, "%c", &data) != EOF)
		while (fscanf_s(fp, "%c", &data, sizeof(data)) != EOF)
		{
			if (data == ' ')
			{
				j++;
			}
			else if (data == '\n')
			{
				i++;
				j = 0;
			}
			else if (data != 13)
			{
				hWallMatrix[i][j] = data - 48;
			}
		}

	fclose(fp);

	fopen_s(&fp, "vwall10.txt", "r");
	i = 0;
	j = 0;

	while (fscanf_s(fp, "%c", &data, sizeof(data)) != EOF)
	{
		if (data == ' ')
		{
			j++;
		}
		else if (data == '\n')
		{
			i++;
			j = 0;
		}
		else if (data != 13)
		{
			vWallMatrix[i][j] = data - 48;
		}
	}
	fclose(fp);
}

void setRobotsMap() {
	/* set costs of maps */
	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			terreinMatrix[ii][jj] = rand() % 200;
		}
	}

	int asdf = 0;
	asdf = rand() % 40 + 50;

	for (int index = 0; index < NUM_ROBOT; index++)
	{
		for (int ii = 0; ii < MAP_SIZE; ii++)
		{
			for (int jj = 0; jj < MAP_SIZE; jj++)
			{
				if (index % 2 == 0)
				{
					robotList[index].travelCost[ii][jj] = terreinMatrix[ii][jj];
				}
				else
				{
					robotList[index].travelCost[ii][jj] = asdf;
				}
			}
		}
	}

	printf("print map\n\n");

	/* print map information */
	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			printf("%4d ", robotList[0].travelCost[ii][jj]);
			if (jj < MAP_SIZE - 1)
			{
				if (vWallMatrix[ii][jj] == 1)
				{
					printf("| ");
				}
				else
				{
					printf("  ");
				}
			}
		}
		printf("\n");
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			if (ii < MAP_SIZE - 1)
			{
				if (hWallMatrix[ii][jj] == 1)
				{
					printf("%4s ", "---");
				}
				else
				{
					printf("%4c ", ' ');
				}
				if (jj < MAP_SIZE - 1)
				{
					printf("o ");
				}
			}
		}
		printf("\n");
	}
	printf("\n\n");

	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			printf("%4d ", robotList[1].travelCost[ii][jj]);
			if (jj < MAP_SIZE - 1)
			{
				if (vWallMatrix[ii][jj] == 1)
				{
					printf("| ");
				}
				else
				{
					printf("  ");
				}
			}
		}
		printf("\n");
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			if (ii < MAP_SIZE - 1)
			{
				if (hWallMatrix[ii][jj] == 1)
				{
					printf("%4s ", "---");
				}
				else
				{
					printf("%4c ", ' ');
				}
				if (jj < MAP_SIZE - 1)
				{
					printf("o ");
				}
			}
		}
		printf("\n");
	}
	printf("\n");
}

void setInitialRobotLocation() {

	int same;
	Coordinate newItem;

	for (int i = 0; i < NUM_ROBOT; i++)
	{
		robotList[i].setRobotNum(i);
	}

	/* initiate robot location */
	for (int ii = 0; ii < NUM_ROBOT; ii++)
	{
		same = 1;
		while (same == 1)
		{
			newItem.x = rand() % MAP_SIZE;
			newItem.y = rand() % MAP_SIZE;
			same = 0;

			for (int jj = 0; jj < ii; jj++)
			{
				if (newItem.x == robotList[jj].robotCoord.x && newItem.y == robotList[jj].robotCoord.y)
				{
					same = 1;
				}
			}
		}

		robotList[ii].robotCoord.x = newItem.x;
		robotList[ii].robotCoord.y = newItem.y;

		printf("move robot %d to (%d, %d)\n", ii, newItem.x, newItem.y);

		robotList[ii].recognizeWalls();
	}
}

void setInitialTaskLocation() {

	int same;
	Coordinate newItem;

	/* spawn tasks */

	for (int ii = 0; ii< NUM_TASK; ii++)
	{
		totalTaskList[ii].taskId = ii;
		totalTaskList[ii].taskCost = INF;
		totalTaskList[ii].taskStatus = IDLE;
		totalTaskList[ii].taskCoord.x = 11;
		totalTaskList[ii].taskCoord.y = 11;
	}

	/* initiate task location */
	for (int ii = 0; ii < NUM_TASK / 2; ii++)
	{
		same = 1;
		while (same == 1)
		{
			newItem.x = rand() % MAP_SIZE;
			newItem.y = rand() % MAP_SIZE;
			same = 0;

			for (int jj = 0; jj < NUM_ROBOT; jj++)
			{
				if (newItem.x == robotList[jj].robotCoord.x && newItem.y == robotList[jj].robotCoord.y)
				{
					same = 1;
				}
			}

			for (int jj = 0; jj < ii; jj++)
			{
				if (newItem.x == totalTaskList[jj].taskCoord.x && newItem.y == totalTaskList[jj].taskCoord.y)
				{
					same = 1;
				}
			}
		}

		totalTaskList[ii].taskCoord.x = newItem.x;
		totalTaskList[ii].taskCoord.y = newItem.y;
		totalTaskList[ii].taskStatus = MOVING;
		printf("move item %d to (%d, %d)\n", ii, newItem.x, newItem.y);
	}
}

void setInitialTaskCost() {


	/* set costs of task */
	int tempCost[2][NUM_TASK] = { 0, };

	for (int ii = 0; ii < NUM_TASK; ii++)
	{
		tempCost[0][ii] = rand() % 200;
		tempCost[1][ii] = rand() % 280;
	}

	for (int index = 0; index < NUM_ROBOT; index++)
	{
		for (int ii = 0; ii < NUM_TASK; ii++)
		{
			robotList[index].taskCost[ii] = tempCost[index % 2][ii];
		}
	}

	/* print task cost information */
	printf("\n");
	printf("task cost\ntask\t");

	for (int ii = 0; ii < NUM_TASK; ii++)
	{
		printf("%d\t", ii);
	}
	printf("\n");

	for (int index = 0; index < NUM_ROBOT; index++)
	{
		printf("r%d :\t", index);
		for (int ii = 0; ii < NUM_TASK; ii++)
		{
			printf("%d\t", robotList[index].taskCost[ii]);
		}
		printf("\n");
	}
	printf("\n");
}

void updateRobotMap(Robot robot) {
	robot.setNodeMap();
	robot.setCostEdge();
	robot.robotGraph.shortestPath(MAP_SIZE * robot.robotCoord.y + robot.robotCoord.x, &robot);
	robot.setItemPath(totalTaskList);
}

void prepareScheduling() {
	for (int i = 0; i < NUM_ROBOT; i++)
	{
		robotList[i].setNodeMap();
		robotList[i].setCostEdge();
		robotList[i].robotGraph.shortestPath(MAP_SIZE * robotList[i].robotCoord.y + robotList[i].robotCoord.x, &robotList[i]);
		robotList[i].setItemPath(totalTaskList);
	}
}


/* 
MAIN 
*/
int main()
{

	srand(time(NULL));

	buildMap();
	setRobotsMap();
	setInitialRobotLocation();
	setInitialTaskLocation();
	setInitialTaskCost();

	
	int same;
	Coordinate newItem;

	int taskProgress[NUM_ROBOT] = { 0, };
	int movingProgress[NUM_ROBOT] = { 0, };

	int reach[NUM_ROBOT] = { 0, };

	int exitCondition = 0;
	int count = 0;
	int time = 0;

	int task_produced = NUM_TASK / 2;
	int time_produced = TIME_MAX / 2;

	/* scheduling */
	//scheduling();

	//for (int i = 0; i < 3; i++) {
		//int taskID = scheduled[i];
		//std::vector <Coordinate> path = pathGeneration(robotList[i].robotCoord, itemCoord[taskID]);
		//robotList[i].assignTask(scheduled[i], path, itemCoord);
	//}
	prepareScheduling();
	scheduling();


	/* start */
	while (time < TIME_MAX && exitCondition == 0)
	{
		/* new task generation */
		if (time == time_produced)
		{
			time_produced += TIME_MAX / NUM_TASK;
			same = 1;
			while (same == 1)
			{
				newItem.x = rand() % MAP_SIZE;
				newItem.y = rand() % MAP_SIZE;

				same = 0;

				for (int jj = 0; jj < NUM_ROBOT; jj++)
				{
					if (newItem.x == robotList[jj].robotCoord.x && newItem.y == robotList[jj].robotCoord.y)
					{
						same = 1;
					}
				}
				for (int jj = 0; jj < task_produced; jj++)
				{
					if (newItem.x == totalTaskList[jj].taskCoord.x && newItem.y == totalTaskList[jj].taskCoord.y)
					{
						same = 1;
					}
				}
			}

			totalTaskList[task_produced].taskCoord.x = newItem.x;
			totalTaskList[task_produced].taskCoord.y = newItem.y;
			totalTaskList[task_produced].taskStatus = MOVING;

			printf("move item %d to (%d, %d)\n", task_produced, newItem.x, newItem.y);

			task_produced++;
			prepareScheduling();
			scheduling();
		}

		/* simulate robot behavior */
		for (int index = 0; index < NUM_ROBOT; index++)
		{
			/* MOVING state */
			if (robotList[index].status == MOVING)
			{
				if (movingProgress[index] == 0)
				{
					movingProgress[index] += robotList[index].getTravelCost();
				}
				if (movingProgress[index] > 0)
				{
					//robotList[index].energy -= robotList[index].getTravelCost();
					//movingProgress[index] -= robotList[index].getTravelCost();
					movingProgress[index] -= 10;
				}
				else
				{
					//robotList[index].energy -= robotList[index].getTravelCost();
				}

				if (reach[index] == 1)
				{
					robotList[index].status = WORKING;
					robotList[index].pathIndex = 0;
					movingProgress[index] = 0;
					reach[index] = 0;
				}
				else if (movingProgress[index] <= 0)
				{
					robotList[index].updatePostion();
					if (robotList[index].atTask() == true)
					{
						reach[index] = 1;
					}
					robotList[index].energy -= robotList[index].getTravelCost();
					bool newWall = robotList[index].recognizeWalls();
					if (newWall == true) {
						for (int i = 0; i < NUM_ROBOT; i++) {
							int valid = pathValidation(robotList[i].itemPath[robotList[i].taskList[robotList[i].currTask].taskId]);
							if (valid != 0) {
								printf("not valid\n");
								updateRobotMap(robotList[i]);
								int taskID = -1;
								int minCost = INF;
								totalTaskList[robotList[i].taskList[robotList[i].currTask].taskId].taskStatus = MOVING;
								for (int j = 0; j < NUM_TASK; j++) {
									if (totalTaskList[j].taskStatus == MOVING) {
										int dst = robotList[i].distance[MAP_SIZE * totalTaskList[j].taskCoord.y + totalTaskList[j].taskCoord.x];
										if (dst < minCost) {
											taskID = j;
											minCost = dst;
										}
									}
								}
								robotList[i].assignTask(totalTaskList[taskID], robotList[i].itemPath[taskID]);
								totalTaskList[taskID].taskStatus = WORKING;
								printf("robot%d is allocated to task%d\n", i, taskID);
							}
						}
					}
				}
				//printf("robot %d is moving towards task %d\n", index, robotList[index].taskList[robotList[index].curr_task].taskId);
			}
			/* WORKING state */
			else if (robotList[index].status == WORKING)
			{
				if (taskProgress[index] == 0)
				{
					taskProgress[index] += robotList[index].getTaskCost();
				}

				if (taskProgress[index] > 0)
				{
					taskProgress[index] -= 10;
					printf("robot %d is working on task %d\n", index, robotList[index].taskList[robotList[index].currTask].taskId);
				}

				if (taskProgress[index] <= 0)
				{
					printf("robot %d finished task %d\n", index, robotList[index].taskList[robotList[index].currTask].taskId);
					printf("robot %d remining energy %d\n", index, robotList[index].energy);
					robotList[index].energy -= robotList[index].getTaskCost();
					doneList[robotList[index].taskList[robotList[index].currTask].taskId] = 1;
					totalTaskList[robotList[index].taskList[robotList[index].currTask].taskId].taskStatus = IDLE;
					robotList[index].currTask++;
					robotList[index].status = IDLE;
					taskProgress[index] = 0;
					prepareScheduling();
					scheduling();
				}
			}
			/* IDLE state */
			else if (robotList[index].status == IDLE)
			{
				//printf("robot %d is idle\n", index);

				if (robotList[index].currTask < NUM_TASK)
				{
					if (robotList[index].taskList[robotList[index].currTask].taskId > -1)
					{
						robotList[index].status = MOVING;
					}
				}
			}
		}
		time++;
		//simulate robot behavior end

		/* end condition */
		count = 0;
		for (int ii = 0; ii < NUM_TASK; ii++)
		{
			if (doneList[ii] == 1)
			{
				count++;
			}
		}
		if (count == NUM_TASK)
		{
			printf("finished task at time %d\n", time);
			exitCondition = 1;
		}

	}

	/* show result */
	for (int index = 0; index < NUM_ROBOT; index++)
	{
		robotList[index].calcCost();

		printf("robot %d remaining energy %d\n", index, robotList[index].energy);
	}

	printf("done list : ");
	for (int ii = 0; ii < NUM_TASK; ii++)
	{
		printf("%d ", doneList[ii]);

	}
	printf("\n");
	getchar();

	return 0;
}