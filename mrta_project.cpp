#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#define MAP_SIZE 10
#define NUM_ROBOT 4
#define NUM_TASK 16
#define MAX_ENERGY 2000
#define TIME_MAX 4000

#define IDLE 0
#define WORKING 1
#define MOVING 2

#define UP 'w'
#define DOWN 's'
#define LEFT 'a'
#define RIGHT 'd'

int hWallMatrix[MAP_SIZE-1][MAP_SIZE];
int vWallMatrix[MAP_SIZE][MAP_SIZE-1];
int terreinMatrix[MAP_SIZE][MAP_SIZE];


class Node
{
public:
	int nodeNum;
	unsigned int nodeCost;	// 코스트

	Node *left = NULL;
	Node *right = NULL;
	Node *up = NULL;
	Node *down = NULL;

	Node(int xx, int yy, int cost)
	{
		nodeCost = cost;
		nodeNum = MAP_SIZE*xx + yy;
	}
	Node()
	{
		nodeCost = 100000;
		nodeNum = -1;
	}
};


/* Class : Coordinate */
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


/* Class : Task */
class Task
{
public:
	Coordinate taskcoord;
	std::vector<Coordinate> path;
	int taskId;
	int taskCost;
};


/* Class : Robot */
class Robot
{
public:

	Robot();

	Coordinate robotcoord;
	Coordinate taskCoord[NUM_TASK];
	int travelCost[MAP_SIZE][MAP_SIZE]; //travel cost of block Coordinate
	int taskCost[NUM_TASK]; // cost of a task performed by this robot
	Task taskList[NUM_TASK];// list of tasks assgined to this robot
	Node Nodemap[MAP_SIZE*MAP_SIZE];

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
	void setNodeMap(Node Nodemap[]);
	void searchDijkstra();

	bool atTask();


};

Robot::Robot() {
	robotcoord.x = 0;
	robotcoord.y = 0;

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

		printf("path assigned from (%d, %d) to (%d, %d)\n", current.x, current.y, next.x, next.y);

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

	getchar();
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

void Robot::setNodeMap(Node Nodemap[])
{
	for (int i = 0; i < MAP_SIZE; i++)
	{
		for (int j = 0; j < MAP_SIZE; j++)
		{
			// Nodempap 배열에 노드 연결
			Nodemap[MAP_SIZE*i + j] = Node(i, j, travelCost[i][j]);
			// 노드간 관계 연결
			if (i == 0) {
				if (j == 0) {
					// 왼쪽 위
					Nodemap[MAP_SIZE*i + j].down = &Nodemap[MAP_SIZE*(i + 1) + j];
					Nodemap[MAP_SIZE*i + j].right = &Nodemap[MAP_SIZE*i + (j + 1)];
				}
				else if (j == MAP_SIZE - 1) {
					// 오른쪽 위
					Nodemap[MAP_SIZE*i + j].down = &Nodemap[MAP_SIZE*(i + 1) + j];
					Nodemap[MAP_SIZE*i + j].left = &Nodemap[MAP_SIZE*i + (j - 1)];
				}
				else {
					Nodemap[MAP_SIZE*i + j].down = &Nodemap[MAP_SIZE*(i + 1) + j];
					Nodemap[MAP_SIZE*i + j].left = &Nodemap[MAP_SIZE*i + (j - 1)];
					Nodemap[MAP_SIZE*i + j].right = &Nodemap[MAP_SIZE*i + (j + 1)];
				}
			}
			else if (i == MAP_SIZE - 1) {
				if (j == 0) {
					// 왼쪽 아래
					Nodemap[MAP_SIZE*i + j].up = &Nodemap[MAP_SIZE*(i + 1) + j];
					Nodemap[MAP_SIZE*i + j].right = &Nodemap[MAP_SIZE*i + (j + 1)];
				}
				else if (j == MAP_SIZE - 1) {
					// 오른쪽 아래
					Nodemap[MAP_SIZE*i + j].up = &Nodemap[MAP_SIZE*(i + 1) + j];
					Nodemap[MAP_SIZE*i + j].left = &Nodemap[MAP_SIZE*i + (j - 1)];
				}
				else {
					Nodemap[MAP_SIZE*i + j].up = &Nodemap[MAP_SIZE*(i + 1) + j];
					Nodemap[MAP_SIZE*i + j].left = &Nodemap[MAP_SIZE*i + (j - 1)];
					Nodemap[MAP_SIZE*i + j].right = &Nodemap[MAP_SIZE*i + (j + 1)];
				}
			}
			else if (j == 0) {
				Nodemap[MAP_SIZE*i + j].up = &Nodemap[MAP_SIZE*(i + 1) + j];
				Nodemap[MAP_SIZE*i + j].down = &Nodemap[MAP_SIZE*i + (j - 1)];
				Nodemap[MAP_SIZE*i + j].right = &Nodemap[MAP_SIZE*i + (j + 1)];
			}
			else if (j == MAP_SIZE - 1) {
				Nodemap[MAP_SIZE*i + j].up = &Nodemap[MAP_SIZE*(i + 1) + j];
				Nodemap[MAP_SIZE*i + j].down = &Nodemap[MAP_SIZE*i + (j - 1)];
				Nodemap[MAP_SIZE*i + j].left = &Nodemap[MAP_SIZE*i + (j + 1)];
			}
			else {
				Nodemap[MAP_SIZE*i + j].up = &Nodemap[MAP_SIZE*(i - 1) + j];
				Nodemap[MAP_SIZE*i + j].down = &Nodemap[MAP_SIZE*(i + 1) + j];
				Nodemap[MAP_SIZE*i + j].left = &Nodemap[MAP_SIZE*i + (j - 1)];
				Nodemap[MAP_SIZE*i + j].right = &Nodemap[MAP_SIZE*i + (j + 1)];
			}
		}
	}

}

void Robot::searchDijkstra()
{

}


/* Class : RobotToTask */
class RobotToTask
{
public:
	int robotID;
	int taskID;
	std::vector <Coordinate> path;
	int cost;
};


/* Class : RobotToTasks */
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
int * scheduling(RobotToTasks * pathes, int robotNum, int taskNum);


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


int * scheduling(RobotToTasks * pathes, int robotNum, int taskNum) {
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

	for (int i = 0; i < robotNum; i++) {
		printf("robot%d is assigned to tast%d\n", i, scheduledRobot[i]);
	}

	return scheduledRobot;

}

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


/* MAIN */


int main()
{
	srand(time(NULL));

	char data;

	// map loading and generating
	FILE  * fp;
	fopen_s(&fp, "hwall10.txt", "r");

	int i = 0;
	int j = 0;

	while(fscanf_s(fp, "%c", &data) != EOF)
	{
		if(data == ' ')
		{
			j++;
		}
		else if(data == '\n')
		{
			i++;
			
			j = 0;
		}
		else if(data != 13)
		{
			hWallMatrix[i][j] = data - 48;
			
		}
	}

	fclose(fp);

	fopen_s(&fp, "vwall10.txt", "r");
	
	i = 0;
	j = 0;

	while(fscanf_s(fp, "%c", &data ) != EOF)
	{
		if(data == ' ')
		{
			j++;
		}
		else if(data == '\n')
		{
			i++;
			j = 0;
		}
		else if(data != 13)
		{
			vWallMatrix[i][j] = data-48;
		}
	}
	fclose(fp);

	Robot robotList[NUM_ROBOT];

	printf("print map\n\n");
	for(int ii = 0; ii < MAP_SIZE; ii++)
	{
		for(int jj = 0; jj < MAP_SIZE; jj++)
		{
			terreinMatrix[ii][jj] = rand()%200;
		}
	}

	int asdf = 0;
	asdf = rand() % 40 + 50;

	for(int index = 0; index < NUM_ROBOT; index++)
	{
		for(int ii = 0; ii < MAP_SIZE; ii++)
		{
			for(int jj = 0; jj < MAP_SIZE; jj++)
			{
				if(index %2 == 0)
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
		
	//spawn tasks
	Coordinate itemCoord[NUM_TASK];
	for(int ii = 0; ii< NUM_TASK; ii ++)
	{
		itemCoord[ii].x = 11;
		itemCoord[ii].y = 11;
	}
	for(int ii = 0; ii < MAP_SIZE; ii++)
	{
		for(int jj = 0; jj < MAP_SIZE; jj++)
		{
			printf("%4d ", robotList[0].travelCost[ii][jj]);
			//fprintf(fp, "%4d ", robotList[0].travelCost[ii][jj]);
			if( jj < MAP_SIZE-1)
			{
				if(vWallMatrix[ii][jj] == 1)
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
		for(int jj = 0; jj < MAP_SIZE; jj++)
		{
			if(ii < MAP_SIZE-1 )
			{
				if(hWallMatrix[ii][jj] == 1)
				{
					printf("%4s ","---");
					//fprintf(fp, "%4s ","---");
				}
				else
				{
					printf("%4c ",' ');
					//fprintf(fp, "%4c ",' ');
				}
				if(jj < MAP_SIZE-1 )
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

	for(int ii = 0; ii < MAP_SIZE; ii++)
	{
		for(int jj = 0; jj < MAP_SIZE; jj++)
		{
			
			printf("%4d ", robotList[1].travelCost[ii][jj]);
			//fprintf(fp, "%4d ", robotList[0].travelCost[ii][jj]);
			if( jj < MAP_SIZE-1)
			{
				if(vWallMatrix[ii][jj] == 1)
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
		for(int jj = 0; jj < MAP_SIZE; jj++)
		{
			if(ii < MAP_SIZE-1 )
			{
				if(hWallMatrix[ii][jj] == 1)
				{
					printf("%4s ","---");
					//fprintf(fp, "%4s ","---");
				}
				else
				{
					printf("%4c ",' ');
					//fprintf(fp, "%4c ",' ');
				}
				if(jj < MAP_SIZE-1 )
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
	
	int doneList[NUM_TASK] = {0,};

	for(int ii = 0 ;ii < NUM_ROBOT; ii++)
	{
		same = 1;
		while(same == 1)
		{
			newItem.x = rand()%MAP_SIZE;
			newItem.y = rand()%MAP_SIZE;
			same = 0;
			
			for(int jj = 0; jj < ii; jj++)
			{
				if(newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
				{
					same = 1;
				}
			}
		}

		robotList[ii].robotcoord.x = newItem.x;
		robotList[ii].robotcoord.y = newItem.y;

		printf("move robot %d to (%d, %d)\n", ii, newItem.x,newItem.y);
	}

	for(int ii = 0; ii < NUM_TASK/2; ii++)
	{
		same = 1;
		while(same == 1)
		{
			newItem.x = rand() % MAP_SIZE;
			newItem.y = rand() % MAP_SIZE;
			same = 0;
			
			for(int jj = 0; jj < NUM_ROBOT; jj++)
			{
				if(newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
				{
					same = 1;
				}
			}

			for(int jj = 0; jj < ii; jj++)
			{
				if(newItem.x == itemCoord[jj].x && newItem.y == itemCoord[jj].y)
				{
					same = 1;
				}	
			}
		}

		itemCoord[ii].x = newItem.x;
		itemCoord[ii].y = newItem.y;

		printf("move item %d to (%d, %d)\n", ii, newItem.x,newItem.y);
	}
	
	int tempCost[2][NUM_TASK] = {0,};

	for(int ii = 0; ii < NUM_TASK; ii++)
	{
		tempCost[0][ii] = rand() % 200;
		tempCost[1][ii] = rand() % 280;
	}

	for(int index = 0; index < NUM_ROBOT; index++)
	{
		for(int ii = 0; ii < NUM_TASK; ii++)
		{
			robotList[index].taskCost[ii] = tempCost[index%2][ii];
		}
	}

	printf("\n");
	printf("task cost\ntask\t");
	
	for(int ii = 0; ii < NUM_TASK; ii++)
	{
		printf("%d\t", ii);
	}
	printf("\n");

	for(int index = 0; index < NUM_ROBOT; index++)
	{
		printf("r%d :\t", index);
		for(int ii = 0; ii < NUM_TASK; ii++)
		{
			printf("%d\t",robotList[index].taskCost[ii]);
		}
		printf("\n");
	}
	printf("\n");

	

	RobotToTasks * test = forTest();
	int * scheduled = scheduling(test, 3, 4);

	for (int i = 0; i < 3; i++) {
		int taskID = scheduled[i];
		std::vector <Coordinate> path = pathGeneration(robotList[i].robotcoord, itemCoord[taskID]);
		robotList[i].assignTask(scheduled[i], path, itemCoord);
	}
	
	//currentPos.x = 1;

	//asdff.push_back(currentPos);

	//currentPos.y = 1;

	//asdff.push_back(currentPos);

	//robotList[0].assignTask(0, path, itemCoord);
	

	int taskProgress[NUM_ROBOT] = {0,};
	int movingProgress[NUM_ROBOT] = {0,};

	int reach[NUM_ROBOT] = {0,};

	int exitCondition = 0;
	int count = 0;
	int time = 0;

	int task_produced = NUM_TASK/2;
	int time_produced = TIME_MAX/2;

	while(time < TIME_MAX && exitCondition == 0)
	{
		if(time == time_produced )
		{
			time_produced += TIME_MAX/NUM_TASK;
			same = 1;
			while(same == 1)
			{
				newItem.x = rand()%MAP_SIZE;
			
				newItem.y = rand()%MAP_SIZE;
			
				same = 0;
			
				for(int jj = 0; jj < NUM_ROBOT; jj++)
				{
					if(newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
					{
						same = 1;
					}
				}
				for(int jj = 0; jj < task_produced; jj++)
				{
					if(newItem.x == itemCoord[jj].x && newItem.y == itemCoord[jj].y)
					{
						same = 1;
					}	
				}
			}

			itemCoord[task_produced].x = newItem.x;
			itemCoord[task_produced].y = newItem.y;

			printf("move item %d to (%d, %d)\n", task_produced, newItem.x,newItem.y);

			task_produced++;
		}
		
		//simulate robot behavior
		
		for(int index = 0; index < NUM_ROBOT; index++)
		{
			if(robotList[index].status == MOVING)
			{
				if(movingProgress[index] == 0)
				{
					movingProgress[index] += robotList[index].getTravelCost();
				}
				if(movingProgress[index] > 0)
				{
					//robotList[index].energy -= robotList[index].getTravelCost();
					//movingProgress[index] -= robotList[index].getTravelCost();
					movingProgress[index] -= 1;
				}
				else
				{
					robotList[index].energy -= robotList[index].getTravelCost();
				}

				if(reach[index] == 1)
				{
					robotList[index].status = WORKING;
					robotList[index].pathIndex = 0;
					movingProgress[index] = 0;
					reach[index] = 0;
				}
				else if(movingProgress[index] <= 0)
				{
					robotList[index].updatePostion();
					if(robotList[index].atTask()== true)
					{
						reach[index] = 1;
					}
				}
				printf("robot %d is moving towards task %d\n", index, robotList[index].taskList[robotList[index].curr_task].taskId);
			}
			else if(robotList[index].status == WORKING)
			{
				if(taskProgress[index] == 0)
				{
					taskProgress[index] += robotList[index].getTaskCost();
				}
				
				if(taskProgress[index] > 0)
				{
					taskProgress[index] -= 1;
					printf("robot %d is working on task %d\n", index, robotList[index].taskList[robotList[index].curr_task].taskId);
				}
				else if(taskProgress[index] <= 0)
				{
					printf("robot %d finished task %d\n", index, robotList[index].taskList[robotList[index].curr_task].taskId);
					robotList[index].energy -= robotList[index].getTaskCost();
					doneList[robotList[index].taskList[robotList[index].curr_task].taskId] = 1;
					robotList[index].curr_task++;
					robotList[index].status = IDLE;
					taskProgress[index] = 0;
				}
			}
			else if(robotList[index].status == IDLE)
			{
				printf("robot %d is idle\n", index);

				if(robotList[index].curr_task < NUM_TASK)
				{
					if(robotList[index].taskList[robotList[index].curr_task].taskId > -1)
					{
						robotList[index].status = MOVING;
					}
				}
			}
		}
		time++;
		//simulate robot behavior end

		// end condition
		count = 0;
		for(int ii = 0; ii < NUM_TASK; ii++)
		{
			if(doneList[ii] == 1)
			{
				count++;
			}
		}
		if(count == NUM_TASK)
		{
			printf("finished task at time %d\n", time);
			exitCondition = 1;
		}
		
	}

	for(int index = 0; index < NUM_ROBOT; index++)
	{
		robotList[index].calcCost();

		printf("robot %d remaining energy %d\n", index, robotList[index].energy);
	}

	printf("done list : ");
	for(int ii = 0; ii < NUM_TASK; ii++)
	{
		printf("%d ", doneList[ii]);
	
	}
	printf("\n");

	//print_result(robotList, 0);
}

