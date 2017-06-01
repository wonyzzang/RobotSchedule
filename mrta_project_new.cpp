#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include <queue>
#include <vector>
#include <functional>
#include <list>

using namespace std;


/* Define constants */

#define INF 1000000
#define MAP_SIZE 10
#define NUM_ROBOT 4
#define NUM_TASK 16
#define MAX_ENERGY 2000
#define TIME_MAX 400

#define UP 'w'
#define DOWN 's'
#define LEFT 'a'
#define RIGHT 'd'

/*
Enumerator : RobotStatus
*/
enum RobotStatus {
	IDLE,  
	WORKING, 
	MOVING  
};

/*
Enumerator : TaskStatus
*/
enum TaskStatus
{
	NOT_SPAWN,
	SPAWN, 
	ALLOCATED, 
	DONE
};

typedef pair<int, int> iPair;


/* Class Definition */

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

/*
Class : Node
For tracking shortest path
*/
class Node
{
public:
	Node(int xx, int yy, int cost);
	Node();

	int nodeNum;
	unsigned int nodeCost;	// cost
	Node *prev = NULL;   // previous node in path
	Coordinate coord;
};

/*
Class : Task
*/
class Task
{
public:
	int taskId;
	int robotId = -1;   // ID of robot that finished task
	Coordinate taskCoord;
	TaskStatus status;
};

/*
Class : Graph
*/
class Graph
{
public:
	int v;    // No. of vertices
			  // In a weighted graph, we need to store vertex
			  // and weight pair for every edge
	list< pair<int, int> > *adj;  // adjacent node

	Graph();
	Graph(int V);  // Constructor
				   // function to add an edge to graph
	void addEdge(int u, int v, int w1, int w2);   // add edge between two vertice
	void deleteEdge(int u, int v, int w);
};

/*
Class : Robot
*/
class Robot
{
public:
	Robot();
	Robot(int id);

	int robotId;   // robot id

	Coordinate robotCoord;  // location of robot
	int travelCost[MAP_SIZE][MAP_SIZE]; //travel cost of block Coordinate
	int taskCost[NUM_TASK]; // cost of a task performed by this robot

	int distance[MAP_SIZE * MAP_SIZE];  // moving cost to node
	int allocatedTask;  // list of tasks assgined to this robot

	Node nodeMap[MAP_SIZE * MAP_SIZE];  // for generating path by using dijkstra's algorithm
	Graph robotGraph;   // converted nodeMap 

	int totalCost[NUM_TASK]; // total cost for task
	vector<Coordinate> currPath;   // current path to move
	vector<Coordinate> taskPath[NUM_TASK];   // paths to tasks

	int totalBlocks; // total number of blocks traveled

	RobotStatus status;
	int energy;   // robot's energy
	int progress;   // indicator of ongoing process like moving, working

	void setNodeMap();    // make nodemap
	void setRobotId(int id);
	
	void setCostEdge();   // make links between nodes
	void setItemPath();   // tracking shortest path to task
	void setItemCost();   // calculate sum of move cost and task cost

	void assignTask(Task task, std::vector<Coordinate> inputPath);

	int getTravelCost();
	int getTravelCost(int x, int y);
	int getTravelCost(Coordinate location);

	int getTaskCost(int taskId);

	Coordinate getCurrentPosition();

	void updatePosition();
	bool recognizeWalls();   // recognize walls around the robot
	bool atTask();

	void shortestPath(int s);   // dijkstra's algorithm
};


/* First global variables definition */

int hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
int terreinMatrix[MAP_SIZE][MAP_SIZE];

int hWallSearch[MAP_SIZE - 1][MAP_SIZE] = { 0, };  // horizontal walls recognized by robots
int vWallSearch[MAP_SIZE][MAP_SIZE - 1] = { 0, };  // vertical walls recognized by robots

Task taskList[NUM_TASK];  // take list
Robot robotList[NUM_ROBOT];  // robot list


/* Function Definition */

int pathValidation(std::vector <Coordinate> inputPath);
void scheduling();  // scheduling based on min-min
void buildMap();   // initialize true map
void setRobotsMap();  // set robot cost map
void setInitialRobotLocation();  // set initial robot's location
void setInitialTaskLocation();  // set initial task's location
void setInitialTaskCost();    // set initial task's cost
void updateRobotMap(Robot * robot);   // update robot nodemap for single robot
void prepareScheduling();   // update robot nodemap for all robots


/* Class Implementation */

/*
Class : Coordinate
*/
Coordinate::Coordinate()
{
	this->x = 0;
	this->y = 0;
}

Coordinate::Coordinate(int xx, int yy)
{
	this->x = xx;
	this->y = yy;
}

/*
Class : Node
*/
Node::Node() {
	this->nodeCost = INF;
	this->nodeNum = -1;
}

Node::Node(int xx, int yy, int cost) {
	this->nodeCost = cost;
	this->nodeNum = MAP_SIZE * xx + yy;
	this->coord = Coordinate(xx, yy);
}

/*
Class : Graph
*/
Graph::Graph()
{
	this->v = MAP_SIZE * MAP_SIZE;
	this->adj = new list<iPair>[v];
}

Graph::Graph(int v)
{
	this->v = v;
	this->adj = new list<iPair>[v];
}

void Graph::addEdge(int u, int v, int w1, int w2)
{
	this->adj[u].push_back(make_pair(v, w1));
}

void Graph::deleteEdge(int u, int v, int w)
{
	this->adj[u].remove(make_pair(v, w));
}

/*
Class : Robot
*/
Robot::Robot() {
	this->robotCoord.x = 0;
	this->robotCoord.y = 0;

	this->robotGraph = Graph(MAP_SIZE * MAP_SIZE);

	this->energy = MAX_ENERGY;

	this->status = IDLE;
	this->progress = 0;
	for (int i = 0; i < NUM_TASK; i++)
	{
		this->taskCost[i] = 0;
	}

	for (int y = 0; y < MAP_SIZE; y++)
	{
		for (int x = 0; x < MAP_SIZE; x++)
		{
			travelCost[y][x] = 0;
		}
	}
	totalBlocks = 0;
}

Robot::Robot(int id) {
	this->robotId = id;
	this->robotCoord.x = 0;
	this->robotCoord.y = 0;
	this->progress = 0;
	this->robotGraph = Graph(MAP_SIZE * MAP_SIZE);

	this->energy = MAX_ENERGY;

	this->status = IDLE;

	for (int i = 0; i < NUM_TASK; i++)
	{
		this->taskCost[i] = 0;
	}

	for (int y = 0; y < MAP_SIZE; y++)
	{
		for (int x = 0; x < MAP_SIZE; x++)
		{
			this->travelCost[y][x] = 0;
		}
	}
	this->totalBlocks = 0;
}

void Robot::setRobotId(int id) {
	this->robotId = id;
}

void Robot::assignTask(Task task, std::vector<Coordinate> inputPath)
{
	Coordinate current;
	Coordinate next;

	current.x = this->robotCoord.x;
	current.y = this->robotCoord.y;

	next.x = task.taskCoord.x;
	next.y = task.taskCoord.y;

	/* print assignment information */
	printf("Robot %d path assigned from (%d, %d) to (%d, %d)\n", this->robotId, current.x, current.y, next.x, next.y);
	printf("assigned path : ");
	for (int i = 0; i < inputPath.size(); i++) {
		Coordinate temp = inputPath.at(i);
		printf("(%d,%d), ", temp.x, temp.y);
	}
	printf("\n");

	/* assign task */
	this->allocatedTask = task.taskId;
	this->currPath = inputPath;

	this->status = MOVING;
	task.status = ALLOCATED;
}

int Robot::getTravelCost()
{
	return this->travelCost[this->robotCoord.y][this->robotCoord.x];
}

int Robot::getTravelCost(Coordinate location)
{
	return this->travelCost[location.y][location.x];
}

int Robot::getTravelCost(int x, int y)
{
	return this->travelCost[y][x];
}

int Robot::getTaskCost(int taskId)
{
	return this->taskCost[taskId];
}

void Robot::setItemCost()
{
	Coordinate tempCoord;
	for (int i = 0; i < NUM_TASK; i++)
	{
		tempCoord = taskList[i].taskCoord;
		int cost = this->taskCost[i] + this->distance[MAP_SIZE * tempCoord.y + tempCoord.x];
		this->totalCost[i] = cost;
	}
}

Coordinate Robot::getCurrentPosition()
{
	return this->robotCoord;
}

bool Robot::atTask()
{
	if (this->robotCoord.x == taskList[this->allocatedTask].taskCoord.x && this->robotCoord.y == taskList[this->allocatedTask].taskCoord.y)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Robot::setNodeMap()
{
	for (int y = 0; y < MAP_SIZE; y++)
	{
		for (int x = 0; x < MAP_SIZE; x++)
		{
			this->nodeMap[MAP_SIZE * y + x].nodeNum = MAP_SIZE * y + x;
			this->nodeMap[MAP_SIZE * y + x].nodeCost = this->travelCost[y][x];
			this->nodeMap[MAP_SIZE * y + x].prev = NULL;
			this->nodeMap[MAP_SIZE * y + x].coord.x = x;
			this->nodeMap[MAP_SIZE * y + x].coord.y = y;
		}
	}

	this->robotGraph = Graph(MAP_SIZE * MAP_SIZE);
}

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
				printf("robot %d (%d,%d) vwall recognized\n", this->robotId, x - 1, y);
				newWall = true;
			}
		}
	}

	if (x != MAP_SIZE - 1) {
		if (vWallMatrix[y][x] == 1) {
			if (vWallSearch[y][x] != 1) {
				vWallSearch[y][x] = 1;
				printf("robot %d (%d,%d) vwall recognized\n", this->robotId, x, y);
				newWall = true;
			}
		}
	}

	/* horizontal walls */
	if (y != 0) {
		if (hWallMatrix[y - 1][x] == 1) {
			if (hWallSearch[y - 1][x] != 1) {
				hWallSearch[y - 1][x] = 1;
				printf("robot %d (%d,%d) hwall recognized\n", this->robotId, x, y - 1);
				newWall = true;
			}
		}
	}

	if (y != MAP_SIZE - 1) {
		if (hWallMatrix[y][x] == 1) {
			if (hWallSearch[y][x] != 1) {
				hWallSearch[y][x] = 1;
				printf("robot %d (%d,%d) hwall recognized\n", this->robotId, x, y);
				newWall = true;
			}
		}
	}

	return newWall;
}

void Robot::setCostEdge()
{
	for (int y = 0; y < MAP_SIZE; y++)
	{
		for (int x = 0; x < MAP_SIZE; x++)
		{
			if (y == 0) {
				if (x == 0) {
					if (vWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x + 1, this->travelCost[y][x + 1], this->travelCost[y][x]);	//right
					}
					if (hWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y + 1) + x, this->travelCost[y + 1][x], this->travelCost[y][x]);	//down
					}
				}
				else if (x == 9) {
					if (vWallSearch[y][x - 1] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x - 1, this->travelCost[y][x - 1], this->travelCost[y][x]);	//left
					}
					if (hWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y + 1) + x, this->travelCost[y + 1][x], this->travelCost[y][x]);//down
					}
				}
				else {
					if (vWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x + 1, this->travelCost[y][x + 1], this->travelCost[y][x]);	//right
					}
					if (vWallSearch[y][x - 1] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x - 1, this->travelCost[y][x - 1], this->travelCost[y][x]);	//left
					}
					if (hWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y + 1) + x, this->travelCost[y + 1][x], this->travelCost[y][x]);//down
					}
				}
			}
			else if (y == 9)
			{
				if (x == 0)
				{
					if (hWallSearch[y - 1][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y - 1) + x, this->travelCost[y - 1][x], this->travelCost[y][x]);//up
					}
					if (vWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x + 1, this->travelCost[y][x + 1], this->travelCost[y][x]);	//right
					}
				}
				else if (x == 9)
				{
					if (hWallSearch[y - 1][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y - 1) + x, this->travelCost[y - 1][x], this->travelCost[y][x]);//up
					}
					if (vWallSearch[y][x - 1] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x - 1, this->travelCost[y][x - 1], this->travelCost[y][x]);	//left
					}
				}
				else
				{
					if (hWallSearch[y - 1][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y - 1) + x, this->travelCost[y - 1][x], this->travelCost[y][x]);//up
					}
					if (vWallSearch[y][x - 1] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x - 1, this->travelCost[y][x - 1], this->travelCost[y][x]);	//left
					}
					if (vWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x + 1, this->travelCost[y][x + 1], this->travelCost[y][x]);	//right
					}
				}
			}
			else
			{
				if (x == 0) {
					if (hWallSearch[y - 1][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y - 1) + x, this->travelCost[y - 1][x], this->travelCost[y][x]);//up
					}
					if (vWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x + 1, this->travelCost[y][x + 1], this->travelCost[y][x]);	//right
					}
					if (hWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y + 1) + x, this->travelCost[y + 1][x], this->travelCost[y][x]);//down
					}
				}
				else if (x == 9) {
					if (hWallSearch[y - 1][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y - 1) + x, this->travelCost[y - 1][x], this->travelCost[y][x]);//up
					}
					if (vWallSearch[y][x - 1] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x - 1, this->travelCost[y][x - 1], this->travelCost[y][x]);	//left
					}
					if (hWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y + 1) + x, this->travelCost[y + 1][x], this->travelCost[y][x]);//down
					}
				}
				else {
					if (hWallSearch[y - 1][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y - 1) + x, this->travelCost[y - 1][x], this->travelCost[y][x]);//up
					}
					if (vWallSearch[y][x - 1] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x - 1, this->travelCost[y][x - 1], this->travelCost[y][x]);	//left
					}
					if (vWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * y + x + 1, this->travelCost[y][x + 1], this->travelCost[y][x]);	//right
					}
					if (hWallSearch[y][x] == 0) {
						this->robotGraph.addEdge(MAP_SIZE * y + x, MAP_SIZE * (y + 1) + x, this->travelCost[y + 1][x], this->travelCost[y][x]);//down
					}
				}
			}
		}
	}
}

void Robot::setItemPath()
{
	Coordinate tempCoord;
	for (int i = 0; i < NUM_TASK; i++)
	{
		this->taskPath[i].clear();
		if (taskList[i].taskCoord.x != 11) {
			/* tracking previous nodes */
			Node node = this->nodeMap[MAP_SIZE * taskList[i].taskCoord.y + taskList[i].taskCoord.x];
			while (node.prev != NULL)
			{
				tempCoord.x = node.coord.x;
				tempCoord.y = node.coord.y;
				this->taskPath[i].push_back(tempCoord);
				node = * node.prev;
				if (this->robotCoord.x == tempCoord.x && this->robotCoord.y == tempCoord.y) {
					break;
				}
			}
		}
	}
}

void Robot::updatePosition()
{
	Coordinate dest = this->currPath.back();
	printf("robot %d move (%d, %d) to (%d, %d)\n", this->robotId, this->robotCoord.x, this->robotCoord.y, dest.x, dest.y);
	this->robotCoord.x = dest.x;
	this->robotCoord.y = dest.y;
	this->currPath.pop_back();
}

void Robot::shortestPath(int src)
{
	priority_queue< iPair, vector <iPair>, greater<iPair> > pq;

	// Create a vector for distances and initialize all
	// distances as infinite (INF)
	vector<int> dist(this->robotGraph.v, INF);

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
		for (i = this->robotGraph.adj[u].begin(); i != this->robotGraph.adj[u].end(); ++i)
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
				this->nodeMap[v].prev = & this->nodeMap[u];
				//printf("%d %d\n", (*robot).Nodemap[v].nodeNum, (*robot).Nodemap[v].prev->nodeNum);
				pq.push(make_pair(dist[v], v));
			}
		}
	}
	for (int i = 0; i < this->robotGraph.v; ++i) {
		this->distance[i] = dist[i];
	}
}

int pathValidation(std::vector <Coordinate> inputPath)
{

	if (inputPath.size() == 0) {
		return 0;
	}

	int pass = 0;

	int xdiff = 0;
	int ydiff = 0;

	int prevx = 0;
	int prevy = 0;
	int nextx = 0;
	int nexty = 0;

	for (int i = 0; i < inputPath.size() - 1; i++)
	{
		nextx = inputPath.at(i + 1).x;
		nexty = inputPath.at(i + 1).y;
		prevx = inputPath.at(i).x;
		prevy = inputPath.at(i).y;

		xdiff = nextx - prevx;
		ydiff = nexty - prevy;

		if (xdiff == -1 && ydiff == 0)
		{
			//left
			if (vWallSearch[prevy][prevx - 1] == 1)
			{
				pass = 4;
			}
		}
		else if (xdiff == 1 && ydiff == 0)
		{
			//right
			if (vWallSearch[prevy][prevx] == 1)
			{
				pass = 4;
			}
		}
		else if (ydiff == -1 && xdiff == 0)
		{
			//up
			if (hWallSearch[prevy - 1][prevx] == 1)
			{
				pass = 4;
			}
		}
		else if (ydiff == 1 && xdiff == 0)
		{
			//down
			if (hWallSearch[prevy][prevx] == 1)
			{
				pass = 4;
			}
		}
		else
		{
			pass = 8;
			printf("jump path (%d, %d), (%d, %d)\n", prevx, prevy, nextx, nexty);
			for (int j = 0; j < inputPath.size() - 1; j++) {
				int pathx = inputPath.at(j).x;
				int pathy = inputPath.at(j).y;
				printf(" path(%d,%d)", pathx, pathy);
			}
			printf("\n");
			break;
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

void scheduling() {
	int targetTasks[NUM_TASK] = { 0, };   // indicator whether this task need allocation or not 
	int scheduleTable[NUM_TASK][NUM_ROBOT];  // table for min-min scheduling
	int robotNeedToSchedule = 0;   // number of idle robot
	int taskNeedToSchedule = 0;   // number of not allocated task

	for (int j = 0; j < NUM_TASK; j++) {
		for (int i = 0; i < NUM_ROBOT; i++) {
			scheduleTable[j][i] = INF;
		}
	}

	/* make scheduling table */
	for (int i = 0; i < NUM_TASK; i++) {
		if (taskList[i].status == SPAWN) {
			printf("task %d cost : ", i);
			taskNeedToSchedule++;
			for (int j = 0; j < NUM_ROBOT; j++) {
				if (robotList[j].status == IDLE) {
					if (robotList[j].energy > robotList[j].totalCost[i]) {
						scheduleTable[i][j] = robotList[j].totalCost[i];
						printf("%d,", scheduleTable[i][j]);
					}
					else {
						printf("NoEnergy,");
					}
				}
			}
			printf("\n");
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

	int scheduledRobot[NUM_ROBOT] = {-1,-1,-1,-1};   // task id allocated
	int scheduledTask[NUM_TASK] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };  // robot id allocated 

	int minCostPerTask[NUM_TASK] = {INF,INF,INF,INF, INF, INF, INF, INF, INF,INF,INF,INF, INF, INF, INF, INF};
	int robotIDs[NUM_TASK] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };  // robot id of minCostPerTask value

	/* find min cost per task */
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

	/* scheduling */
	for (int i = 0; i < robotNeedToSchedule; i++) {

		int taskID = -1;
		int minCost = INF;

		/* find min cost task and robot */
		for (int j = 0; j < NUM_TASK; j++) {
			if (scheduledTask[j] == -1) {
				if (minCostPerTask[j] < minCost) {
					taskID = j;
					minCost = minCostPerTask[j];
				}
			}
		}

		if (taskID == -1) {
			printf("There is no task to schedule\n");
			break;
		}

		/* assign task */
		int selectedRobot = robotIDs[taskID];
		scheduledRobot[selectedRobot] = taskID;
		scheduledTask[taskID] = selectedRobot;
		printf("robot%d is allocated to task%d cost %d\n", selectedRobot, taskID, minCost);
		minCostPerTask[taskID] = INF;
		taskList[taskID].status = ALLOCATED;
		robotList[selectedRobot].assignTask(taskList[taskID], robotList[selectedRobot].taskPath[taskID]);

		/* update minCostPerTask */
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
}

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

	std::fclose(fp);

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
	std::fclose(fp);
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
		for (int y = 0; y < MAP_SIZE; y++)
		{
			for (int x = 0; x < MAP_SIZE; x++)
			{
				if (index % 2 == 0)
				{
					robotList[index].travelCost[y][x] = terreinMatrix[y][x];
				}
				else
				{
					robotList[index].travelCost[y][x] = asdf;
				}
			}
		}
	}

	printf("print map\n\n");

	/* print map information */
	for (int y = 0; y < MAP_SIZE; y++)
	{
		for (int x = 0; x < MAP_SIZE; x++)
		{
			printf("%4d ", robotList[0].travelCost[y][x]);
			if (x < MAP_SIZE - 1)
			{
				if (vWallMatrix[y][x] == 1)
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
		for (int x = 0; x < MAP_SIZE; x++)
		{
			if (y < MAP_SIZE - 1)
			{
				if (hWallMatrix[y][x] == 1)
				{
					printf("%4s ", "---");
				}
				else
				{
					printf("%4c ", ' ');
				}
				if (x < MAP_SIZE - 1)
				{
					printf("o ");
				}
			}
		}
		printf("\n");
	}
	printf("\n\n");

	for (int y = 0; y < MAP_SIZE; y++)
	{
		for (int x = 0; x < MAP_SIZE; x++)
		{
			printf("%4d ", robotList[1].travelCost[y][x]);
			if (x < MAP_SIZE - 1)
			{
				if (vWallMatrix[y][x] == 1)
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
		for (int x = 0; x < MAP_SIZE; x++)
		{
			if (y < MAP_SIZE - 1)
			{
				if (hWallMatrix[y][x] == 1)
				{
					printf("%4s ", "---");
				}
				else
				{
					printf("%4c ", ' ');
				}
				if (x < MAP_SIZE - 1)
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

	/* initiate robot location */
	for (int i = 0; i < NUM_ROBOT; i++)
	{
		same = 1;
		while (same == 1)
		{
			newItem.x = rand() % MAP_SIZE;
			newItem.y = rand() % MAP_SIZE;
			same = 0;

			for (int j = 0; j < i; j++)
			{
				if (newItem.x == robotList[j].robotCoord.x && newItem.y == robotList[j].robotCoord.y)
				{
					same = 1;
				}
			}
		}

		robotList[i].robotCoord.x = newItem.x;
		robotList[i].robotCoord.y = newItem.y;

		printf("move robot %d to (%d, %d)\n", i, newItem.x, newItem.y);

		robotList[i].recognizeWalls();
	}
}

void setInitialTaskLocation() {

	int same;
	Coordinate newItem;

	/* spawn tasks */
	for (int i = 0; i< NUM_TASK; i++)
	{
		taskList[i].taskId = i;
		taskList[i].status = NOT_SPAWN;
		taskList[i].taskCoord.x = 11;
		taskList[i].taskCoord.y = 11;
	}

	/* initiate task location */
	for (int i = 0; i < NUM_TASK / 2; i++)
	{
		same = 1;
		while (same == 1)
		{
			newItem.x = rand() % MAP_SIZE;
			newItem.y = rand() % MAP_SIZE;
			same = 0;

			for (int j = 0; j < NUM_ROBOT; j++)
			{
				if (newItem.x == robotList[j].robotCoord.x && newItem.y == robotList[j].robotCoord.y)
				{
					same = 1;
				}
			}

			for (int j = 0; j < i; j++)
			{
				if (newItem.x == taskList[j].taskCoord.x && newItem.y == taskList[j].taskCoord.y)
				{
					same = 1;
				}
			}
		}

		taskList[i].taskCoord.x = newItem.x;
		taskList[i].taskCoord.y = newItem.y;
		taskList[i].status = SPAWN;
		printf("item %d is located  to (%d, %d)\n", i, newItem.x, newItem.y);
	}
}

void setInitialTaskCost() {

	/* set costs of task */
	int tempCost[2][NUM_TASK] = { 0, };

	for (int i = 0; i < NUM_TASK; i++)
	{
		tempCost[0][i] = rand() % 200;
		tempCost[1][i] = rand() % 280;
	}

	for (int index = 0; index < NUM_ROBOT; index++)
	{
		for (int i = 0; i < NUM_TASK; i++)
		{
			robotList[index].taskCost[i] = tempCost[index % 2][i];
		}
	}

	/* print task cost information */
	printf("\n");
	printf("task cost\ntask\t");

	for (int i = 0; i < NUM_TASK; i++)
	{
		printf("%d\t", i);
	}
	printf("\n");

	for (int index = 0; index < NUM_ROBOT; index++)
	{
		printf("r%d :\t", index);
		for (int i = 0; i < NUM_TASK; i++)
		{
			printf("%d\t", robotList[index].taskCost[i]);
		}
		printf("\n");
	}
	printf("\n");
}

void updateRobotMap(Robot *robot) {
	(*robot).setNodeMap();
	(*robot).setCostEdge();
	(*robot).shortestPath(MAP_SIZE * (*robot).robotCoord.y + (*robot).robotCoord.x);
	(*robot).setItemPath();
	(*robot).setItemCost();
}

void prepareScheduling() {
	for (int i = 0; i < NUM_ROBOT; i++)
	{
		robotList[i].setNodeMap();
		robotList[i].setCostEdge();
		robotList[i].shortestPath(MAP_SIZE * robotList[i].robotCoord.y + robotList[i].robotCoord.x);
		robotList[i].setItemPath();
		robotList[i].setItemCost();
	}
}


/* 
MAIN 
*/
int main()
{
	/* setting */
	srand(time(NULL));
	buildMap();

	for (int i = 0; i < NUM_ROBOT; i++)
	{
		robotList[i] = Robot(i);
	}

	setRobotsMap();
	setInitialRobotLocation();
	setInitialTaskLocation();
	setInitialTaskCost();

	
	int same;
	Coordinate newItem;

	int reach[NUM_ROBOT] = { 0, };

	int exitCondition = 0;
	int count = 0;
	int time = 0;

	int task_produced = NUM_TASK / 2;
	int time_produced = TIME_MAX / 2;

	/* first scheduling */
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

				for (int j = 0; j < NUM_ROBOT; j++)
				{
					if (newItem.x == robotList[j].robotCoord.x && newItem.y == robotList[j].robotCoord.y)
					{
						same = 1;
					}
				}
				for (int j = 0; j < task_produced; j++)
				{
					if (newItem.x == taskList[j].taskCoord.x && newItem.y == taskList[j].taskCoord.y)
					{
						same = 1;
					}
				}
			}

			taskList[task_produced].taskCoord.x = newItem.x;
			taskList[task_produced].taskCoord.y = newItem.y;
			taskList[task_produced].status = SPAWN;

			printf("item %d is located to (%d, %d)\n", task_produced, newItem.x, newItem.y);

			/* task spawn scheduling */
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
				int progress = robotList[index].progress;

				if (progress == 0) {
					if (!robotList[index].currPath.empty()) {
						robotList[index].progress += robotList[index].getTravelCost(robotList[index].currPath.back());
					}
					else {
						printf("robot %d reach task's location\n", index);
					}
					
				}
				
				if (progress > 0)
				{
					robotList[index].progress -= 10;
				}

				progress = robotList[index].progress;
				if (reach[index] == 1)
				{
					robotList[index].status = WORKING;
					robotList[index].progress = 0;
					reach[index] = 0;
				}
				else if (progress <= 0)
				{
					robotList[index].updatePosition();
					if (robotList[index].atTask() == true)
					{
						reach[index] = 1;
					}
					robotList[index].energy -= robotList[index].getTravelCost();
					robotList[index].totalCost[robotList[index].allocatedTask] -= robotList[index].getTravelCost();
					printf("robot %d consume energy %d remining energy %d\n", index, robotList[index].getTravelCost(), robotList[index].energy);
					bool newWall = robotList[index].recognizeWalls();
					
					/* path checking */
					if (newWall == true) {
						for (int i = 0; i < NUM_ROBOT; i++) {
							Robot robot = robotList[i];
							if (robot.status == IDLE) {
								continue;
							}
							printf("robot %d path validation dest:task %d\n", robotList[i].robotId, robotList[i].allocatedTask);
							int valid = pathValidation(robotList[i].currPath);
							if (valid != 0) {
								int preCost = robotList[i].totalCost[robotList[i].allocatedTask];
								int preTask = robotList[i].allocatedTask;
								updateRobotMap(&robotList[i]);
								int postCost = robotList[i].totalCost[robotList[i].allocatedTask];
								
								/* change task or not */
								if (postCost > 2 * preCost) {
									printf("postCost %d > 2 * preCost %d\n", postCost, preCost);
									int taskID = -1;
									int minCost = INF;
									taskList[robotList[i].allocatedTask].status = SPAWN;
									for (int j = 0; j < NUM_TASK; j++) {
										if (taskList[j].status == SPAWN) {
											int dst = robotList[i].totalCost[j];
											if (dst < minCost) {
												taskID = j;
												minCost = dst;
											}
										}
									}
									robotList[i].assignTask(taskList[taskID], robotList[i].taskPath[taskID]);
									taskList[taskID].status = ALLOCATED;
									printf("wall warning - robot%d is allocated to task%d preTask %d\n", i, taskID, preTask);
								}
								else {
									printf("postCost %d <= 2 * preCost %d\n", postCost, preCost);
									robotList[i].assignTask(taskList[preTask], robotList[i].taskPath[preTask]);
								}
								
							}
						}
					}
				}
			}
			/* WORKING state */
			else if (robotList[index].status == WORKING)
			{
				int progress = robotList[index].progress;
				if (progress == 0)
				{
					robotList[index].progress += robotList[index].getTaskCost(robotList[index].allocatedTask);
				}

				if (progress > 0)
				{
					robotList[index].progress -= 10;
					printf("robot %d is working on task %d\n", index, robotList[index].allocatedTask);
				}
				progress = robotList[index].progress;
				if (progress <= 0)
				{
					printf("robot %d finished task %d\n", index, robotList[index].allocatedTask);
					robotList[index].energy -= robotList[index].getTaskCost(robotList[index].allocatedTask);
					printf("robot %d consume energy %d remining energy %d\n", index, robotList[index].getTaskCost(robotList[index].allocatedTask), robotList[index].energy);
					taskList[robotList[index].allocatedTask].status = DONE;
					taskList[robotList[index].allocatedTask].robotId = index;
					robotList[index].status = IDLE;
					robotList[index].progress = 0;

					/* task finishing scheduling */
					prepareScheduling();
					scheduling();
				}
			}
		}
		time++;
		//simulate robot behavior end

		/* end condition */
		count = 0;
		for (int i = 0; i < NUM_TASK; i++)
		{
			if (taskList[i].status == DONE)
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
		printf("robot %d remaining energy %d\n", index, robotList[index].energy);
	}

	printf("done list : ");
	for (int i = 0; i < NUM_TASK; i++)
	{
		printf("%d ", taskList[i].status == DONE ? 1 : 0 );
	}
	printf("\n");

	printf("robot id list : ");
	for (int i = 0; i < NUM_TASK; i++)
	{
		printf("%d ", taskList[i].robotId);
	}
	printf("\n");
	getchar();

	return 0;
}