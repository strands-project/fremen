#include <stdlib.h>
#include "ros/ros.h"
#include "CFrelementSet.h"
#include <actionlib/server/simple_action_server.h>
#include <fremenserver/FremenAction.h>

using namespace std;

ros::NodeHandle *n;

typedef actionlib::SimpleActionServer<fremenserver::FremenAction> Server;
Server* server;
fremenserver::FremenResult result;
fremenserver::FremenFeedback feedback;

bool debug = false;

CFrelementSet frelements;

void actionServerCallback(const fremenserver::FremenGoalConstPtr& goal, Server* as)
{
	std::stringstream mess;

	/*check if the element id is not too long*/
	if (goal->id.size() > 1000){
		result.success = false;
		result.message = "State ID is longer than 100 characters, choose a shorter ID.";
		server->setAborted(result);
		return;
	}
	if (goal->order > NUM_PERIODICITIES)
	{
		result.success = false;
		result.message = "Model order is too high. Reasonable value is 2, maximum is NUM_PERIODICITIES.";
		server->setAborted(result);
		return;
	}

	if (debug) ROS_DEBUG("Command received %s %s\n",goal->action.c_str(),goal->id.c_str());

	/*perform model update (if needed)*/
	if (goal->action == "update")
	{
		if (frelements.update(goal->id.c_str(),goal->order)){
			result.message = "Fremen model updated";
			result.success = true;
			server->setSucceeded(result);
		}else{
			result.message = "Fremen model updated";
			result.success = true;
			server->setAborted(result);
		}
	}
	else if (goal->action == "add")
	{
		result.success = true;
		if (frelements.add(goal->id.c_str(),(uint32_t*)goal->times.data(),(unsigned char*)goal->states.data(),(int)goal->states.size()))
		{
			mess << "Added " << (int)goal->states.size() << " measurements to the state " << goal->id;
    			result.message = mess.str(); 
		}else{
			mess << "A new state " <<  goal->id << " was added to the collection and filled with "  << (int)goal->states.size() << " measurements.";
    			result.message = mess.str(); 
		}
		server->setSucceeded(result);
	}	
	else if (goal->action == "predict")
	{
		float probabilities[goal->times.size()];
		result.success = frelements.estimate(goal->id.c_str(),(uint32_t*)goal->times.data(),probabilities,(int)goal->times.size(),goal->order);
		if (result.success)
		{
			mess << "Performed " << (int)goal->times.size() << " predictions of the state " << goal->id;
			result.probability.assign(probabilities,probabilities + (int)goal->times.size());
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "State ID " << goal->id << " does not exist.";
			result.message = mess.str();
			server->setAborted(result);
		}
	}
	else if (goal->action == "evaluate")
	{
		float evaluations[goal->order+1];
		int bestOrder = frelements.evaluate(goal->id.c_str(),(uint32_t*)goal->times.data(),(unsigned char*)goal->states.data(),(int)goal->times.size(),goal->order,evaluations);
		if (bestOrder >= 0)
		{
			mess << "Performed " <<  (goal->order+1) << " evaluations of the model "  << goal->id << " using " << (int)goal->times.size() << " ground truth values. The best performing model has order " << bestOrder;
			result.success = true;
			result.message = mess.str();
			result.errors.assign(evaluations,evaluations + goal->order+1);
			server->setSucceeded(result);
		}else{
			mess << "State ID " << goal->id << " does not exist.";
			result.message = mess.str();
			result.success = false;
			server->setAborted(result);
		}
	}
	else if (goal->action == "remove")
	{
		int num = frelements.remove(goal->id.c_str());
		if (num > 0)
		{
			mess << "State ID " << goal->id << " removed from the collection of " << num << " states.";
			result.message = mess.str();
			result.success = true;
			server->setSucceeded(result);
		}else{
			mess << "State ID " << goal->id << " does not exist.";
			result.message = mess.str();
			result.success = false;
			server->setAborted(result);
		}
	}

	else if (goal->action == "debug")
	{
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
	CFrelementSet frelements;
	uint32_t 	times[100000];
	unsigned char 	state[100000];
	float 		probsA[100000];
	float 		probsB[100000];
	int len = 7*24*60;
	FILE* file = fopen("input.txt","r");
	int dummy=0;
	int dummy2=0;
	for (int i = 0;i<len;i++)
	{
		dummy2 = fscanf(file,"%i\n",&dummy);
		state[i] = dummy;
		times[i] = 60*i;
	}
	fclose(file);

	frelements.add("A",times,state,len);
	frelements.estimate("A",times,probsA,len,2);

	int granul = 7;
	for (int i = 0;i<2;i++) frelements.add("B",&times[i*len/granul],&state[i*len/granul],len/granul);
	for (int i = 0;i<granul;i++) frelements.estimate("B",&times[i*len/granul],&probsB[i*len/granul],len/granul,2);

	frelements.print(true);
	file = fopen("output.txt","w");
	
	for (int i = 0;i<len;i++)fprintf(file,"%i %.3f %.3f\n",state[i],probsA[i],probsB[i]);
	fclose(file);
}

int main(int argc,char* argv[])
{
	//test();
	//return 0;
	ros::init(argc, argv, "fremenserver");
	n = new ros::NodeHandle();
	server = new Server(*n, "/fremenserver", boost::bind(&actionServerCallback, _1, server), false);
	server->start();

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
	return 0;
}
