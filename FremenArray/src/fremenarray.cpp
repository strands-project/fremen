#include <stdlib.h>
#include "ros/ros.h"
#include "CFrelement.h"
#include <actionlib/server/simple_action_server.h>
#include <fremenarray/FremenArrayAction.h>

using namespace std;

ros::NodeHandle *n;

typedef actionlib::SimpleActionServer<fremenarray::FremenArrayAction> Server;
Server* server;
fremenarray::FremenArrayResult result;
fremenarray::FremenArrayFeedback feedback;

bool debug = false;

CFrelement *frelementArray = NULL;
float *values = NULL;
uint32_t times[1];
int numStates = 0;
bool stop = false;


void actionServerCallback(const fremenarray::FremenArrayGoalConstPtr& goal, Server* as)
{
	std::stringstream mess;

	/*check if the element id is not too long*/
	if (goal->order <0 || goal->order > NUM_PERIODICITIES)
	{
		result.success = false;
		result.message = "Model order is out of bounds high. Reasonable value is 2, maximum is NUM_PERIODICITIES, minimum 0.";
		server->setAborted(result);
		return;
	}
//	if (debug) ROS_DEBUG("Command received %s %s\n",goal->operation.c_str(),goal->id.c_str());

	result.probabilities.clear();
	result.entropies.clear();
	result.errors.clear();
	times[0] = goal->time;
	if (goal->operation == "add")
	{
		if (frelementArray == NULL)
		{
			numStates = goal->states.size();
			frelementArray =  new CFrelement[numStates]; 
			values =  (float*) malloc(sizeof(float)*numStates);
		}
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
	else if (goal->operation == "predict")
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
	}
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "fremenarray");
	n = new ros::NodeHandle();
	server = new Server(*n, "/fremenarray", boost::bind(&actionServerCallback, _1, server), false);
	server->start();

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
	ros::shutdown();
	delete server;
	delete n;
	delete[] frelementArray;
	free(values);
	return 0;
}
