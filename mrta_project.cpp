#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <queue>
#include <functional>
#include <list>
#include <windows.h>

using namespace std;
#define INF 10000
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

<<<<<<< HEAD
int hWallMatrix[MAP_SIZE-1][MAP_SIZE];
int vWallMatrix[MAP_SIZE][MAP_SIZE-1];
int terreinMatrix[MAP_SIZE][MAP_SIZE];

<<<<<<< HEAD
int hWallSearch[MAP_SIZE - 1][MAP_SIZE] = { 0, };
int vWallSearch[MAP_SIZE][MAP_SIZE - 1] = { 0, };

/* 
Class : Coordinate 
*/
=======
=======
typedef pair<int, int> iPair;
>>>>>>> origin/qbranch

int hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
int terreinMatrix[MAP_SIZE][MAP_SIZE];




/* Class : Coordinate */
>>>>>>> origin/qbranch
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

class Node
{
public:
	int nodeNum;
	unsigned int nodeCost;	// 코스트
	Node *prev = NULL;
	Coordinate coord;

	Node(int xx, int yy, int cost)
	{
		nodeCost = cost;
		nodeNum = MAP_SIZE*xx + yy;
		Coordinate coord = Coordinate(xx, yy);
	}
	Node()
	{
		nodeCost = 100000;
		nodeNum = -1;
	}
};


/* 
Class : Task 
*/
class Task
{
public:
	Coordinate taskcoord;
	std::vector<Coordinate> path;
	int taskId;
	int taskCost;
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
		int V;    // No. of vertices

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

	int robotnum;
	void setrobotnum(int i);
	Coordinate robotcoord;
	Coordinate taskCoord[NUM_TASK];
	int travelCost[MAP_SIZE][MAP_SIZE]; //travel cost of block Coordinate
	int taskCost[NUM_TASK]; // cost of a task performed by this robot
	int distance[MAP_SIZE*MAP_SIZE];
	Task taskList[NUM_TASK];// list of tasks assgined to this robot

	Node Nodemap[MAP_SIZE*MAP_SIZE];
	vector<Coordinate> Itempath[NUM_TASK];
	void setNodemap();

	Graph robotgraph;
	void setCostEdge();
	void setItemPath(Coordinate Itemcoord[]);

	int totalCost;// total energy consumed
	int totalBlocks; // total number of blocks traveled

	int num_task;
	int status;
	int energy;

	int curr_task;
	int pathIndex;

	void assignTask(int taskId, std::vector<Coordinate> inputPath, Coordinate itemList[]);

	void calcCost();

	int getTravelCost();
	int getTravelCost(int x, int y);

	int getTaskCost();
	int getTaskCost(int x);

	Coordinate getCurrentPosition();

	void updatePostion();
<<<<<<< HEAD
<<<<<<< HEAD
	void recognizeWalls();
=======
	void setNodeMap(Node Nodemap[]);
	void searchDijkstra();
>>>>>>> origin/qbranch

=======
>>>>>>> origin/qbranch
	bool atTask();

};

Robot::Robot() {
	robotcoord.x = 0;
	robotcoord.y = 0;

	Graph robotGraph = Graph(MAP_SIZE*MAP_SIZE);
	
	num_task = 0;

	energy = MAX_ENERGY;

	status = IDLE;
	curr_task = 0;
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


void Robot::assignTask(int taskId, std::vector<Coordinate> inputPath, Coordinate itemList[])
{
	Coordinate current;
	Coordinate next;

	int pass = 0;

	if (num_task < NUM_TASK)
	{
		if (num_task == 0)
		{
			current.x = robotcoord.x;
			current.y = robotcoord.y;
		}
		else
		{
			current.x = itemList[taskList[num_task - 1].taskId].x;
			current.y = itemList[taskList[num_task - 1].taskId].y;
		}

		next.x = itemList[taskId].x;
		next.y = itemList[taskId].y;

		printf("Robot %d path assigned from (%d, %d) to (%d, %d)\n", robotnum , current.x, current.y, next.x, next.y);

		//pass = pathValidation(inputPath, current, next);

		if (pass == 0)
		{
			taskList[num_task].taskId = taskId;
			taskList[num_task].path = inputPath;
			taskList[num_task].taskcoord.x = next.x;
			taskList[num_task].taskcoord.y = next.y;
			num_task++;
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

	for (int ii = 0; ii < num_task; ii++)
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
	return travelCost[robotcoord.y][robotcoord.x];
}

int Robot::getTravelCost(int x, int y)
{
	return travelCost[y][x];
}

int Robot::getTaskCost()
{
	return taskCost[taskList[curr_task].taskId];
}

int Robot::getTaskCost(int x)
{
	return taskCost[x];
}

Coordinate Robot::getCurrentPosition()
{
	return robotcoord;
}

bool Robot::atTask()
{
	if (robotcoord.x == taskList[curr_task].taskcoord.x && robotcoord.y == taskList[curr_task].taskcoord.y)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Robot::setrobotnum(int i)
{
	robotnum = i;
}

void Robot::setNodemap()
{
	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			Nodemap[MAP_SIZE*ii + jj].nodeNum = MAP_SIZE*ii + jj;
			Nodemap[MAP_SIZE*ii + jj].nodeCost = travelCost[ii][jj];
			Nodemap[MAP_SIZE*ii + jj].coord.x = jj;
			Nodemap[MAP_SIZE*ii + jj].coord.y = ii;
		}
	}
}

<<<<<<< HEAD
<<<<<<< HEAD
/* 
Function : recognizeWalls
recognize walls around the robot 
*/
void Robot::recognizeWalls() 
{
	int x = robotcoord.x;
	int y = robotcoord.y;

	/* vertical walls */
	if (x != 0) {
		if (vWallMatrix[y][x - 1] == 1) {
			if (vWallSearch[y][x - 1] != 1) {
				vWallSearch[y][x - 1] = 1;
				printf("(%d,%d) vwall recognized\n", x - 1, y);
			}
		}
	}

	if (x != MAP_SIZE-1) {
		if (vWallMatrix[y][x] == 1) {
			if (vWallSearch[y][x] != 1) {
				vWallSearch[y][x] = 1;
				printf("(%d,%d) vwall recognized\n", x, y);
			}
		}
	}

	/* horizontal walls */
	if (y != 0) {
		if (hWallMatrix[y-1][x] == 1) {
			if (hWallSearch[y - 1][x] != 1) {
				hWallSearch[y - 1][x] = 1;
				printf("(%d,%d) hwall recognized\n", x, y - 1);
			}
		}
	}

	if (y != MAP_SIZE - 1) {
		if (hWallMatrix[y][x] == 1) {
			if (hWallSearch[y][x] != 1) {
				hWallSearch[y][x] = 1;
				printf("(%d,%d) hwall recognized\n", x, y);
			}
		}
	}
}

/* 
Class : Node
*/
class Node
{
public:
	Node();
	Node(int xx, int yy, int cost);

	Coordinate pos;	//	좌표계 설정
	unsigned int nodeCost;	// 코스트

	Node *left;
	Node *right;
	Node *up;
	Node *down;
};
=======
void Robot::setNodeMap(Node Nodemap[])
=======
void Robot::setCostEdge()
>>>>>>> origin/qbranch
{
	for (int ii = 0; ii < MAP_SIZE; ii++) 
	{
		for (int jj = 0; jj < MAP_SIZE; jj++) 
		{
			if (ii == 0) {
				if (jj == 0) {
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj + 1, travelCost[ii][jj+1], travelCost[ii][jj]);	//right
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii+1) + jj, travelCost[ii+1][jj], travelCost[ii][jj]);	//down
					//printf("%d %d type 1\n", ii, jj);
				}
				else if (jj == 9) {
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					//printf("%d %d type 2\n", ii, jj);
				}
				else{
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					//printf("%d %d type 3\n", ii, jj);
				}
			}
			else if (ii == 9)
			{
				if (jj == 0)
				{
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					//printf("%d %d type 4\n", ii, jj);
				}
				else if (jj == 9)
				{
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					//printf("%d %d type 5\n", ii, jj);

				}
				else
				{
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					//printf("%d %d type 6\n", ii, jj);
				}
			}
			else
			{
				if (jj == 0) {
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					//printf("%d %d type 7\n", ii, jj);
				}
				else if (jj == 9) {
					
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					//printf("%d %d type 8\n", ii, jj);
				}
				else {
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj - 1, travelCost[ii][jj - 1], travelCost[ii][jj]);	//left
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*ii + jj + 1, travelCost[ii][jj + 1], travelCost[ii][jj]);	//right
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii - 1) + jj, travelCost[ii - 1][jj], travelCost[ii][jj]);//up
					robotgraph.addEdge(MAP_SIZE*ii + jj, MAP_SIZE*(ii + 1) + jj, travelCost[ii + 1][jj], travelCost[ii][jj]);//down
					//printf("%d %d type 9\n", ii, jj);
				}
			}
		}
	}
}

void Robot::setItemPath(Coordinate Itemcoord[])
{
	Coordinate tempCoord;
	for (int i = 0; i < NUM_TASK; i++)
	{
		// printf("Itemcoord input %d %d \n", Itemcoord[i].x, Itemcoord[i].y);
		if (Itemcoord[i].x != 11) {
			Itempath[i].push_back(Itemcoord[i]);
			while (Nodemap[MAP_SIZE*Itemcoord[i].y + Itemcoord[i].x].prev != NULL)
			{
				tempCoord.x = Nodemap[MAP_SIZE*Itemcoord[i].y + Itemcoord[i].x].coord.x;
				tempCoord.y = Nodemap[MAP_SIZE*Itemcoord[i].y + Itemcoord[i].x].coord.y;
				Itempath[i].push_back(tempCoord);
				Nodemap[MAP_SIZE*Itemcoord[i].y + Itemcoord[i].x] = *Nodemap[MAP_SIZE*Itemcoord[i].y + Itemcoord[i].x].prev;
				//printf("Node <%d %d> \n", tempCoord.y, tempCoord.x);
			}

		}
	}
}

void Robot::updatePostion()
{
	pathIndex++;

	if (pathIndex < taskList[curr_task].path.size())
	{
		robotcoord.x = taskList[curr_task].path.at(pathIndex).x;
		robotcoord.y = taskList[curr_task].path.at(pathIndex).y;
	}
	else
	{
		pathIndex = 0;
	}
}

// Allocates memory for adjacency list
Robot::Graph::Graph()
{
	this->V = MAP_SIZE*MAP_SIZE;
	adj = new list<iPair>[V];
}
Robot::Graph::Graph(int V)
{
	this->V = V;
	adj = new list<iPair>[V];
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
	vector<int> dist(V, INF);

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
				(*robot).Nodemap[v].prev = &(*robot).Nodemap[u];
				//printf("%d %d\n", (*robot).Nodemap[v].nodeNum, (*robot).Nodemap[v].prev->nodeNum);
				pq.push(make_pair(dist[v], v));
			}
		}
	}
	// Print shortest distances stored in dist[]
	//printf("Vertex   Distance from Source\n");
	for (int i = 0; i < V; ++i)
		//printf("%d \t\t %d\n", i, dist[i]);
		(*robot).distance[i] = dist[i] ;
	Node copymap[MAP_SIZE*MAP_SIZE];
	memcpy(&copymap, &(*robot).Nodemap, sizeof((*robot).Nodemap));
	//Node point = *copymap;
	/*for (int i = 0; i < V; i++) 
	{
		Node point = copymap[i];
		printf("%d : ", copymap[i].nodeNum);
		while (point.prev != NULL) {
			printf("%d ", point.prev->nodeNum);
			point = *point.prev;
		}
		printf("\n");
	}
	*/
}

>>>>>>> origin/qbranch

Node::Node(int xx, int yy, int cost) {
	pos.x = xx;
	pos.y = yy;
	nodeCost = cost;
}

<<<<<<< HEAD
Node::Node() {
	pos.x = 0;
	pos.y = 0;
	nodeCost = 0;
}


/* 
Class : RobotToTask 
*/
=======

/* Class : RobotToTask */
>>>>>>> origin/qbranch
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
RobotToTasks * forTest();
int * scheduling(RobotToTasks * pathes, int robotNum, int taskNum, Robot robotlist[]);


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
		printf("path (%d,%d)\n", pathx, pathy);
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

<<<<<<< HEAD

/* 
Function : pathGeneration
temporary function to generate path 
*/
=======
vector <Coordinate> pathrev(vector <Coordinate> temppath) {
	vector <Coordinate> path;
	vector <Coordinate> path_rev = temppath;

	for (int i = 0; i < temppath.size(); i++) {
		path.push_back(temppath.back());
		temppath.pop_back();
	}
	
	return path;
}

>>>>>>> origin/qbranch
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

<<<<<<< HEAD
/* 
Function : scheduling
perform min-min scheduling (need to revise later)
*/
int * scheduling(RobotToTasks * pathes, int robotNum, int taskNum) {
=======

int * scheduling(RobotToTasks * pathes, int robotNum, int taskNum, Robot robotlist[]) {
>>>>>>> origin/qbranch
	int * scheduledRobot = new int[robotNum];
	int * scheduledTask = new int[taskNum];
	int ** scheduleTable = new int *[taskNum];

	int * minCostPerTask = new int[taskNum];
	int * robotIDs = new int[taskNum];

	// initiate scheduled robot array
	for (int i = 0; i < robotNum; i++) {
		scheduledRobot[i] = -1;
	}

	// initiate scheduled task array
	for (int j = 0; j < taskNum; j++) {
		scheduledTask[j] = -1;
	}

	// make table for scheduling
	for (int j = 0; j < taskNum; j++) {
		scheduleTable[j] = new int[robotNum];
	}

	for (int i = 0; i < robotNum; i++) {
		RobotToTasks curRobot = pathes[i];
		for (int j = 0; j < taskNum; j++) {
			RobotToTask task = curRobot.list[j];
			scheduleTable[j][i] = task.cost;
		}
	}

	// find min cost per task
	for (int j = 0; j < taskNum; j++) {
		int robotID = -1;
		int minCost = 1000000;

		for (int i = 0; i < robotNum; i++)
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
	for (int i = 0; i < robotNum; i++) {
		for (int j = 0; j < taskNum; j++) {
			printf("%d cost task %d\n", minCostPerTask[j], j);
		}

		int taskID = -1;
		int minCost = 1000000;

		// find min cost task and robot
		for (int j = 0; j < taskNum; j++) {
			if (scheduledTask[j] == -1) {
				if (minCostPerTask[j] < minCost) {
					taskID = j;
					minCost = minCostPerTask[j];
				}
			}
		}

		int selectedRobot = robotIDs[taskID];
		scheduledRobot[selectedRobot] = taskID;
		scheduledTask[taskID] = selectedRobot;
		printf("robot%d is allocated to task%d\n", selectedRobot, taskID);

		// update minCostPerTask
		for (int j = 0; j < taskNum; j++) {
			if (scheduledTask[j] == -1) {
				if (selectedRobot == robotIDs[j]) {
					int robotID = -1;
					int minCost = 1000000;

					for (int i = 0; i < robotNum; i++)
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

	for (int i = 0; i < robotNum; i++)
	{
		printf("robot %d is assigned to task %d with total cost %d \n", i, scheduledRobot[i], robotlist[i].distance[MAP_SIZE*robotlist[i].taskCoord[scheduledRobot[i]].y + robotlist[i].taskCoord[scheduledRobot[i]].x]);
	}

	return scheduledRobot;

}

/* 
Function : forTest
temporary function for test 
*/
RobotToTasks * forTest() {
	Coordinate ro1, ro2, ro3;
	Coordinate t1, t2, t3, t4;

	ro1.x = 7;
	ro1.y = 5;
	ro2.x = 3;
	ro2.y = 4;
	ro3.x = 1;
	ro3.y = 8;

	t1.x = 5;
	t1.y = 9;
	t2.x = 1;
	t2.y = 2;
	t3.x = 4;
	t3.y = 2;
	t4.x = 8;
	t4.y = 9;

	RobotToTasks * result = new RobotToTasks[3];

	RobotToTask * l1 = new RobotToTask[4];
	RobotToTask r1t1;
	r1t1.cost = 100;
	l1[0] = r1t1;
	RobotToTask r1t2;
	r1t2.cost = 130;
	l1[1] = r1t2;
	RobotToTask r1t3;
	r1t3.cost = 60;
	l1[2] = r1t3;
	RobotToTask r1t4;
	r1t4.cost = 80;
	l1[3] = r1t4;
	RobotToTasks r1;
	r1.list = l1;

	RobotToTask * l2 = new RobotToTask[4];
	RobotToTask r2t1;
	r2t1.cost = 70;
	l2[0] = r2t1;
	RobotToTask r2t2;
	r2t2.cost = 50;
	l2[1] = r2t2;
	RobotToTask r2t3;
	r2t3.cost = 90;
	l2[2] = r2t3;
	RobotToTask r2t4;
	r2t4.cost = 100;
	l2[3] = r2t4;
	RobotToTasks r2;
	r2.list = l2;

	RobotToTask * l3 = new RobotToTask[4];
	RobotToTask r3t1;
	r3t1.cost = 120;
	l3[0] = r3t1;
	RobotToTask r3t2;
	r3t2.cost = 90;
	l3[1] = r3t2;
	RobotToTask r3t3;
	r3t3.cost = 80;
	l3[2] = r3t3;
	RobotToTask r3t4;
	r3t4.cost = 70;
	l3[3] = r3t4;
	RobotToTasks r3;
	r3.list = l3;

	result[0] = r1;
	result[1] = r2;
	result[2] = r3;

	return result;

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

<<<<<<< HEAD
	while (fscanf_s(fp, "%c", &data) != EOF)
=======
	while (fscanf_s(fp, "%c", &data, sizeof(data)) != EOF)
>>>>>>> origin/qbranch
	{
		if (data == ' ')
		{
			j++;
		}
		else if (data == '\n')
		{
			i++;
<<<<<<< HEAD
=======

>>>>>>> origin/qbranch
			j = 0;
		}
		else if (data != 13)
		{
			hWallMatrix[i][j] = data - 48;
<<<<<<< HEAD
=======

>>>>>>> origin/qbranch
		}
	}

	fclose(fp);

	fopen_s(&fp, "vwall10.txt", "r");
<<<<<<< HEAD
	i = 0;
	j = 0;
	while (fscanf_s(fp, "%c", &data) != EOF)
=======

	i = 0;
	j = 0;

	while (fscanf_s(fp, "%c", &data, sizeof(data)) != EOF)
>>>>>>> origin/qbranch
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


/* MAIN */

int main()
{
	srand(time(NULL));

	buildMap();

	Robot robotList[NUM_ROBOT];
	for (int i = 0; i < NUM_ROBOT; i++)
	{
		robotList[i].setrobotnum(i);
	}

	printf("print map\n\n");
<<<<<<< HEAD

	/* set costs of maps */
	for(int ii = 0; ii < MAP_SIZE; ii++)
=======
	for (int ii = 0; ii < MAP_SIZE; ii++)
>>>>>>> origin/qbranch
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

<<<<<<< HEAD
	/* spawn tasks */
=======
	//spawn tasks
>>>>>>> origin/qbranch
	Coordinate itemCoord[NUM_TASK];
	for (int ii = 0; ii < NUM_TASK; ii++)
	{
		itemCoord[ii].x = 11;
		itemCoord[ii].y = 11;
	}
<<<<<<< HEAD

	/* print map information */
	for(int ii = 0; ii < MAP_SIZE; ii++)
=======
	for (int ii = 0; ii < MAP_SIZE; ii++)
>>>>>>> origin/qbranch
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			printf("%4d ", robotList[0].travelCost[ii][jj]);
			//fprintf(fp, "%4d ", robotList[0].travelCost[ii][jj]);
			if (jj < MAP_SIZE - 1)
			{
				if (vWallMatrix[ii][jj] == 1)
				{
					printf("| ");
					//fprintf(fp, "| ");
				}
				else
				{
					printf("  ");
					//fprintf(fp, "  ");
				}
			}
		}
		printf("\n");
		//fprintf(fp, "\n");
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			if (ii < MAP_SIZE - 1)
			{
				if (hWallMatrix[ii][jj] == 1)
				{
					printf("%4s ", "---");
					//fprintf(fp, "%4s ","---");
				}
				else
				{
					printf("%4c ", ' ');
					//fprintf(fp, "%4c ",' ');
				}
				if (jj < MAP_SIZE - 1)
				{
					printf("o ");
					//fprintf(fp, "o ");
				}
			}
		}
		printf("\n");
		//	fprintf(fp, "\n");
	}
	printf("\n\n");
	//fprintf(fp,"\n");

	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{

			printf("%4d ", robotList[1].travelCost[ii][jj]);
			//fprintf(fp, "%4d ", robotList[0].travelCost[ii][jj]);
			if (jj < MAP_SIZE - 1)
			{
				if (vWallMatrix[ii][jj] == 1)
				{
					printf("| ");
					//fprintf(fp, "| ");
				}
				else
				{
					printf("  ");
					//fprintf(fp, "  ");
				}
			}
		}
		printf("\n");
		//fprintf(fp, "\n");
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			if (ii < MAP_SIZE - 1)
			{
				if (hWallMatrix[ii][jj] == 1)
				{
					printf("%4s ", "---");
					//fprintf(fp, "%4s ","---");
				}
				else
				{
					printf("%4c ", ' ');
					//fprintf(fp, "%4c ",' ');
				}
				if (jj < MAP_SIZE - 1)
				{
					printf("o ");
					//fprintf(fp, "o ");
				}
			}
		}
		printf("\n");
		//	fprintf(fp, "\n");
	}
	printf("\n");

	int same;

	Coordinate newItem;

<<<<<<< HEAD
	/* initiate robot location */
	for(int ii = 0 ;ii < NUM_ROBOT; ii++)
=======
	int doneList[NUM_TASK] = { 0, };

	for (int ii = 0; ii < NUM_ROBOT; ii++)
>>>>>>> origin/qbranch
	{
		same = 1;
		while (same == 1)
		{
			newItem.x = rand() % MAP_SIZE;
			newItem.y = rand() % MAP_SIZE;
			same = 0;

			for (int jj = 0; jj < ii; jj++)
			{
				if (newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
				{
					same = 1;
				}
			}
		}

		robotList[ii].robotcoord.x = newItem.x;
		robotList[ii].robotcoord.y = newItem.y;

<<<<<<< HEAD
		printf("move robot %d to (%d, %d)\n", ii, newItem.x,newItem.y);

		robotList[ii].recognizeWalls();
	}

	/* initiate task location */
	for(int ii = 0; ii < NUM_TASK/2; ii++)
=======
		printf("move robot %d to (%d, %d)\n", ii, newItem.x, newItem.y);
	}

	for (int ii = 0; ii < NUM_TASK / 2; ii++)
>>>>>>> origin/qbranch
	{
		same = 1;
		while (same == 1)
		{
			newItem.x = rand() % MAP_SIZE;
			newItem.y = rand() % MAP_SIZE;
			same = 0;

			for (int jj = 0; jj < NUM_ROBOT; jj++)
			{
				if (newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
				{
					same = 1;
				}
			}

			for (int jj = 0; jj < ii; jj++)
			{
				if (newItem.x == itemCoord[jj].x && newItem.y == itemCoord[jj].y)
				{
					same = 1;
				}
			}
		}

		itemCoord[ii].x = newItem.x;
		itemCoord[ii].y = newItem.y;

		printf("move item %d to (%d, %d)\n", ii, newItem.x, newItem.y);
	}
<<<<<<< HEAD
	
	/* set costs of task */
	int tempCost[2][NUM_TASK] = {0,};
=======
>>>>>>> origin/qbranch

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

<<<<<<< HEAD
	/* scheduling */
=======
	for (int i = 0; i < NUM_ROBOT; i++) 
	{
		robotList[i].setNodemap();
		robotList[i].setCostEdge();
		robotList[i].robotgraph.shortestPath(MAP_SIZE * robotList[i].robotcoord.y + robotList[i].robotcoord.x, &robotList[i]);
		robotList[i].setItemPath(itemCoord);
	}


>>>>>>> origin/qbranch
	RobotToTasks * test = forTest();

	int * scheduled = scheduling(test, 3, 4, robotList);
	for (int i = 0; i < 3; i++) {
		int taskID = scheduled[i];
		//std::vector <Coordinate> path = pathGeneration(robotList[i].robotcoord, itemCoord[taskID]);
		vector <Coordinate> path[NUM_ROBOT][NUM_TASK];
		for (int i = 0; i < NUM_ROBOT; i++) {
			for (int j = 0; j < NUM_TASK; j++) {
				path[i][j] = pathrev(robotList[i].Itempath[j]);
			}

		}

		robotList[i].assignTask(scheduled[i], path[i][0], itemCoord);
	}
<<<<<<< HEAD
	
	int taskProgress[NUM_ROBOT] = {0,};
	int movingProgress[NUM_ROBOT] = {0,};
=======

	//currentPos.x = 1;

	//asdff.push_back(currentPos);

	//currentPos.y = 1;

	//asdff.push_back(currentPos);

	//robotList[0].assignTask(0, path, itemCoord);

>>>>>>> origin/qbranch

	int taskProgress[NUM_ROBOT] = { 0, };
	int movingProgress[NUM_ROBOT] = { 0, };

	int reach[NUM_ROBOT] = { 0, };

	int exitCondition = 0;
	int count = 0;
	int time = 0;

	int task_produced = NUM_TASK / 2;
	int time_produced = TIME_MAX / 2;

<<<<<<< HEAD
	/* start */
	while(time < TIME_MAX && exitCondition == 0)
	{
		/* new task generation */
		if(time == time_produced )
=======
	while (time < TIME_MAX && exitCondition == 0)
	{
		Sleep(200);
		if (time == time_produced)
>>>>>>> origin/qbranch
		{
			time_produced += TIME_MAX / NUM_TASK;
			same = 1;
			while (same == 1)
			{
				newItem.x = rand() % MAP_SIZE;
<<<<<<< HEAD
				newItem.y = rand() % MAP_SIZE;
			
=======

				newItem.y = rand() % MAP_SIZE;

>>>>>>> origin/qbranch
				same = 0;

				for (int jj = 0; jj < NUM_ROBOT; jj++)
				{
					if (newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
					{
						same = 1;
					}
				}
				for (int jj = 0; jj < task_produced; jj++)
				{
					if (newItem.x == itemCoord[jj].x && newItem.y == itemCoord[jj].y)
					{
						same = 1;
					}
				}
			}

			itemCoord[task_produced].x = newItem.x;
			itemCoord[task_produced].y = newItem.y;

			printf("move item %d to (%d, %d)\n", task_produced, newItem.x, newItem.y);

			task_produced++;
		}

<<<<<<< HEAD
		/* simulate robot behavior */
		for(int index = 0; index < NUM_ROBOT; index++)
		{
			/* MOVING state */
			if(robotList[index].status == MOVING)
=======
		//simulate robot behavior

		for (int index = 0; index < NUM_ROBOT; index++)
		{
			if (robotList[index].status == MOVING)
>>>>>>> origin/qbranch
			{
				if (movingProgress[index] == 0)
				{
					movingProgress[index] += robotList[index].getTravelCost();
				}
				if (movingProgress[index] > 0)
				{
					//robotList[index].energy -= robotList[index].getTravelCost();
					//movingProgress[index] -= robotList[index].getTravelCost();
					movingProgress[index] -= 1;
				}
				else
				{
					robotList[index].energy -= robotList[index].getTravelCost();
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
					robotList[index].recognizeWalls();
				}
				//printf("robot %d is moving towards task %d\n", index, robotList[index].taskList[robotList[index].curr_task].taskId);
			}
<<<<<<< HEAD
			/* WORKING state */
			else if(robotList[index].status == WORKING)
=======
			else if (robotList[index].status == WORKING)
>>>>>>> origin/qbranch
			{
				if (taskProgress[index] == 0)
				{
					taskProgress[index] += robotList[index].getTaskCost();
				}

				if (taskProgress[index] > 0)
				{
					taskProgress[index] -= 1;
					printf("robot %d is working on task %d\n", index, robotList[index].taskList[robotList[index].curr_task].taskId);
				}
				else if (taskProgress[index] <= 0)
				{
					printf("robot %d finished task %d\n", index, robotList[index].taskList[robotList[index].curr_task].taskId);
					robotList[index].energy -= robotList[index].getTaskCost();
					doneList[robotList[index].taskList[robotList[index].curr_task].taskId] = 1;
					robotList[index].curr_task++;
					robotList[index].status = IDLE;
					taskProgress[index] = 0;
				}
			}
<<<<<<< HEAD
			/* IDLE state */
			else if(robotList[index].status == IDLE)
=======
			else if (robotList[index].status == IDLE)
>>>>>>> origin/qbranch
			{
				printf("robot %d is idle\n", index);

				if (robotList[index].curr_task < NUM_TASK)
				{
					if (robotList[index].taskList[robotList[index].curr_task].taskId > -1)
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

<<<<<<< HEAD
	/* show result */
	for(int index = 0; index < NUM_ROBOT; index++)
=======
	for (int index = 0; index < NUM_ROBOT; index++)
>>>>>>> origin/qbranch
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
<<<<<<< HEAD
	getchar();
=======

	//print_result(robotList, 0);

	system("pause");
>>>>>>> origin/qbranch
}

