#include <stdlib.h>
#include "ros/ros.h"
#include "CFrelement.h"
#include "CFrelement2DGridSet.h"
#include <actionlib/server/simple_action_server.h>
#include <fremen2dgrid/Fremen2DGridAction.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

using namespace std;

float *periods = NULL;
ros::NodeHandle *n;

typedef actionlib::SimpleActionServer<fremen2dgrid::Fremen2DGridAction> Server;
Server* server;
fremen2dgrid::Fremen2DGridResult result;
fremen2dgrid::Fremen2DGridFeedback feedback;

bool debug = false;
uint32_t testTime = 0;
CFrelement *frelementArray = NULL;
float *values = NULL;
uint32_t times[1];
bool stop = false;
CFrelement2DGrid *gridA;
CFrelement2DGrid *gridB;
ros::Publisher pubMap;
int predictOrder = 2;

CFrelement2DGridSet* grids; 
nav_msgs::OccupancyGrid predictedMap;
int numArrays = 0;
char mapName[100];

void actionServerCallback(const fremen2dgrid::Fremen2DGridGoalConstPtr& goal, Server* as)
{
	std::stringstream mess;

	//check if the element id is not too long
	if (goal->order <0 || goal->order > NUM_PERIODICITIES)
	{
		result.success = false;
		result.message = "Model order is out of bounds high. Reasonable value is 2, maximum is NUM_PERIODICITIES, minimum 0.";
		server->setAborted(result);
		return;
	}
//	if (debug) ROS_DEBUG("Command received %s %s\n",goal->operation.c_str(),goal->id.c_str());

	times[0] = goal->time;
	nav_msgs::OccupancyGrid *mapPtr = (nav_msgs::OccupancyGrid*)&goal->map;
	if (goal->operation == "add")
	{
		int resultCode = grids->add(goal->mapName.c_str(),goal->time,mapPtr);
		if (resultCode == 1){
			mess << "New map " << goal->mapName << " was added to the map collection.";
			result.message = mess.str(); 
			server->setSucceeded(result);
		}
		else if (resultCode == 0){
			mess << "Map " << goal->mapName << " was updated with measurement from time " << goal->time << ".";
			result.message = mess.str(); 
			server->setSucceeded(result);
		}
		else if (resultCode == -1){
			mess << "Map " << goal->mapName << " has different resolution or dimensions that the one you want to add.";
			result.message = mess.str(); 
			server->setAborted(result);
		}
	}
	else if (goal->operation == "predict")
	{
		int resultCode = grids->estimate(goal->mapName.c_str(),goal->time,mapPtr,goal->order);
		if (resultCode >= 0){
			mess << "Predicted the state of " << goal->mapName << " for time " << goal->time << " using FreMEn order " << goal->order;
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "Map " << goal->mapName << " was not in the map collection ";
			result.message = mess.str(); 
			server->setAborted(result);
		}
	}
	else if (goal->operation == "entropy")
	{
		int resultCode = grids->estimateEntropy(goal->mapName.c_str(),goal->time,mapPtr,goal->order);
		if (resultCode >= 0){
			mess << "Predicted the uncertainty (entropy) of " << goal->mapName << " for time " << goal->time << " using FreMEn order " << goal->order;
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "Map " << goal->mapName << " was not in the map collection ";
			result.message = mess.str(); 
			server->setAborted(result);
		}
	}
	else if (goal->operation == "evaluate")
	{
		float errors[goal->order+1];
		int resultCode = grids->evaluate(goal->mapName.c_str(),goal->time,mapPtr,goal->order,errors);
		if (resultCode >= 0){
			mess << "Evaluated the predictability of " << goal->mapName << " for time " << goal->time << " using FreMEn orders 0 to " << goal->order;
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "Map " << goal->mapName << " was not in the map collection ";
			result.message = mess.str(); 
			server->setAborted(result);
		}
	}
	else if (goal->operation == "print")
	{
		grids->print();
		result.success = true;
		result.message = "Debug printed";
		server->setSucceeded(result);
	}
	else
	{
		result.success = false;		
		result.message = "Unknown action requested";
		server->setAborted(result);
	}
}

int test()
{
	periods = (float*)calloc(NUM_PERIODICITIES,sizeof(float));
	for (int i=0;i<NUM_PERIODICITIES;i++) periods[i] = (24*3600)/(i+1); 
	gridA = new CFrelement2DGrid("A");
	gridB = new CFrelement2DGrid("B");
	//int8_t states[] = {30,0,100,-1, -1,-1,-1,100, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};
	int8_t *states = (int8_t*) calloc(1000000,1);

	printf("Add0\n");
	for (int i = 0;i<1000000;i++) states[i] = 0;
	int dim = 1000;
	gridA->add(0,states,dim,dim,0,0,1);

	printf("Add1\n");
	for (int i = 0;i<1000000;i++) states[i] = 100;
	gridA->add(3600,states,dim,dim,0,0,1);

	printf("Add2\n");
	for (int i = 0;i<1000000;i++) states[i] = 0;
	gridA->add(7200,states,dim,dim,0,0,1);

	printf("Add3\n");
	for (int i = 0;i<1000000;i++) states[i] = 100;
	gridA->add(10800,states,dim,dim,0,0,1);
	//gridA->print(1);

	gridA->estimate(1900,states,1);
	printf("Estimate\n");
	for (int i = 0;i<1;i+=1) printf("%i\n",states[i]);

	printf("Save\n");
	gridA->save("test.grid");

	gridB->load("test.grid");
	gridB->estimate(1900,states,1);
	for (int i = 0;i<1;i+=1) printf("%i\n",states[i]);
	//gridB->print(0);
	free(periods);
	delete gridA;
	delete gridB;
	free(states);
}

void mapCallback(const nav_msgs::OccupancyGrid &msg)
{
	int8_t *data = (int8_t*) msg.data.data();
	//grids->add(msg.header.frame_id.c_str(),msg.info.map_load_time.sec,(nav_msgs::OccupancyGrid*)&msg);
	float errors[5];
	int bestO = grids->evaluate(mapName,testTime,(nav_msgs::OccupancyGrid*)&msg,4,errors);
	printf("Best model %i %f\n",bestO,errors[bestO]);
	if (errors[bestO] < 0.025){ //0.03 without recency
		int result = grids->add(mapName,testTime,(nav_msgs::OccupancyGrid*)&msg);
		if (result ==  0) printf("Map %s updated with info from %i\n",mapName,testTime);
		if (result ==  1) printf("New map %s created from time %i\n",mapName,testTime);
		if (result == -1) printf("Map %s dimension mismatch\n",mapName); 
		if (result >= 0) predictedMap = msg;
		grids->estimate(mapName,testTime,&predictedMap,0);
	}
	pubMap.publish(predictedMap);
}

void nameCallback(const std_msgs::String &msg)
{
	strcpy(mapName,msg.data.c_str());
}

void predictTimeCallback(const std_msgs::Int64 &msg)
{
	grids->estimate(mapName,(int32_t)msg.data,&predictedMap,predictOrder);
	pubMap.publish(predictedMap);
}

void predictOrderCallback(const std_msgs::Int64 &msg)
{
	predictOrder = (int)msg.data;
}

void mapSaveCallback(const std_msgs::String &msg)
{
	grids->save(mapName,msg.data.c_str());
}

void addTimeCallback(const std_msgs::Int64 &msg)
{
	testTime = (int32_t)msg.data;
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "fremen2dgrid");
	n = new ros::NodeHandle();
	server = new Server(*n, "/fremenarray", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	periods = (float*)calloc(NUM_PERIODICITIES,sizeof(float));
	for (int i=0;i<NUM_PERIODICITIES;i++) periods[i] = (24*3600)/(i+1); 
	grids = new CFrelement2DGridSet();
	strcpy(mapName,"default");
	if (argc > 1) grids->load(mapName,argv[1]); 
	ros::Subscriber subMap = n->subscribe("/map", 1, mapCallback);
	ros::Subscriber subPredTime = n->subscribe("/predictTime", 1, predictTimeCallback);
	ros::Subscriber subPredOrder = n->subscribe("/predictOrder", 1, predictOrderCallback);
	ros::Subscriber subAddTime = n->subscribe("/addTime", 1, addTimeCallback);
	ros::Subscriber subName = n->subscribe("/mapName", 1, nameCallback);
	ros::Subscriber subSAve = n->subscribe("/mapSave", 1, mapSaveCallback);
	pubMap = n->advertise<nav_msgs::OccupancyGrid>("/predictedMap", 1);

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
	ros::shutdown();
	delete server;
	delete n;
	return 0;
}
