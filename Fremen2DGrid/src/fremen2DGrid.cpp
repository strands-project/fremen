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
int numStates = 0;
bool stop = false;
CFrelement2DGrid *gridA;
CFrelement2DGrid *gridB;
ros::Publisher pubMap;

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
/*	if (goal->operation == "add")
	{
		numStates = goal->states.size();
		frelementArray =  new CFrelement[numStates]; 
		values =  (float*) malloc(sizeof(float)*numStates);
		if (numStates == goal->states.size())
		{
			int addedStates = 0;
			for (int i = 0;i<numStates;i++){
				if (goal->states[i] != -1){
					values[0] = 0.01*goal->states[i];
					addedStates+=frelementArray[i].add(times,values,1);
				}
			}
			mess << "Added " << addedStates << " states measured at time " << goal->time << "."; 
			result.message = mess.str(); 
			server->setSucceeded(result);
		}else{
			mess << "The length of the 'states' (" << goal->states.size() << ") array does not match the previous size (" << numStates << ").";
			result.message = mess.str(); 
			result.success = -1;
			server->setAborted(result);
		}
	}
	*/
/*	else if (goal->operation == "predict")
	{
			for (int i = 0;i<numStates;i++){
			       	frelementArray[i].estimate(times,&values[i],1,goal->order);
				values[i] = (int)(values[i]*100);
			}
			mess << "Performed " << (int)numStates << " predictions for time " << times[0] << " with FreMEn order " << goal->order;
			result.probabilities.assign(values,values + (int)numStates);
			result.message = mess.str();
			result.success = 0;
			server->setSucceeded(result);
	}
	else if (goal->operation == "entropy")
	{
			for (int i = 0;i<numStates;i++) frelementArray[i].estimateEntropy(times,&values[i],1,goal->order);
			mess << "Performed " << (int)numStates << " entropy predictions for time " << times[0] << " with FreMEn order " << goal->order;
			result.entropies.assign(values,values + (int)numStates);
			result.message = mess.str();
			result.success = 0;
			server->setSucceeded(result);
	}
	else if (goal->operation == "evaluate")
	{
		float errors[goal->order+1];
		float tmpErrors[goal->order+1];
		float states[1];
		if (numStates == goal->states.size())
		{
			for (int j = 0;j<=goal->order;j++) errors[j]=0;
			int addedStates = 0;
			for (int i = 0;i<numStates;i++)
			{
				if (goal->states[i] != -1){
					states[0] = 0.01*goal->states[i];
					addedStates++;
					frelementArray[i].evaluate(times,states,1,goal->order,tmpErrors);
					for (int j = 0;j<=goal->order;j++)errors[j]+=tmpErrors[j];
				}
			}
			for (int j = 0;j<=goal->order;j++) errors[j]=errors[j]/addedStates;
			
			//get best model order
			float error = 10.0;
			int index = 0;
			for (int j = 0;j<=goal->order;j++)
			{
				if (errors[j] < error-0.001){
					index = j;
					error = errors[j]; 
				}
			}
			result.success = index;
			mess << "Performed " <<  (goal->order+1) << " evaluations of each of "  << addedStates << " states to their ground truth values. The best performing model has order " << result.success;
			result.message = mess.str();
			result.errors.assign(errors,errors + goal->order+1);
			server->setSucceeded(result);
		}else{
			mess << "The length of the 'states' (" << goal->states.size() << ") array does not match the previous size (" << numStates << ").";
			result.message = mess.str(); 
			result.success = -1;
			server->setAborted(result);
		}
	}
	else if (goal->operation == "print")
	{
		for (int i = 0;i<numStates;i++) frelementArray[i].print(goal->order);
		result.success = true;
		result.message = "Debug printed";
		server->setSucceeded(result);
	}
	else
	{
		result.success = false;		
		result.message = "Unknown action requested";
		server->setAborted(result);
	}*/
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
	int result = grids->add(mapName,testTime,(nav_msgs::OccupancyGrid*)&msg);
	if (result == 0) printf("Map %s updated with info from %i\n",mapName,testTime); else printf("New map %s created from time %i\n",mapName,testTime);
	predictedMap = msg;
	pubMap.publish(predictedMap);
	testTime += 1800;
}

void nameCallback(const std_msgs::String &msg)
{
	strcpy(mapName,msg.data.c_str());
}

void timeCallback(const std_msgs::Int64 &msg)
{
	grids->estimate(mapName,(int32_t)msg.data,&predictedMap,1);
	pubMap.publish(predictedMap);
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
	ros::Subscriber subMap = n->subscribe("/map", 1, mapCallback);
	ros::Subscriber subTime = n->subscribe("/predictTime", 1, timeCallback);
	ros::Subscriber subName = n->subscribe("/mapName", 1, nameCallback);
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
