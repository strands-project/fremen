#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

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
	if (debug) ROS_DEBUG("Command received %s %s\n",goal->operation.c_str(),goal->id.c_str());

	result.probabilities.clear();
	result.entropies.clear();
	result.errors.clear();
	/*perform model update (if needed)*/
	if (goal->operation == "update")
	{
		if (frelements.update(goal->id.c_str(),goal->order)>=0){
			result.message = "Fremen model updated";
			result.success = true;
			server->setSucceeded(result);
		}else{
			result.message = "Fremen model updated";
			result.success = true;
			server->setAborted(result);
		}
	}
	else if (goal->operation == "detect")
	{
		if (goal->times.size() == goal->values.size()){
			float anomVals[goal->values.size()];
			uint32_t anomTimes[goal->values.size()];
			result.success = frelements.detect(goal->id.c_str(),(uint32_t*)goal->times.data(),(float*)goal->values.data(),(int)goal->values.size(),goal->order,goal->confidence,anomTimes,anomVals);
			if (result.success >=0)
			{
				mess << "Detected " << result.success << " anomalies in " << (int)goal->values.size() << " provided measurements to the state " << goal->id;
				result.message = mess.str(); 
			}else{
				mess << "The state " <<  goal->id << " does not exist in the collection of states.";
				result.message = mess.str(); 
			}
			result.anomalyTimes.assign(anomTimes,anomTimes + result.success);
			result.anomalyValues.assign(anomVals,anomVals + result.success);
			server->setSucceeded(result);
		}else{
			mess << "The length of the 'states' and 'times' arrays does not match.";
			result.message = mess.str(); 
			result.success = -2;
			server->setAborted(result);
		}
	}
	else if (goal->operation == "add")
	{
		if (goal->times.size() == goal->states.size()){
			float values[goal->states.size()];
			for (int i=0;i<goal->states.size();i++){
				if (goal->states[i]) values[i] = 1; else values[i] = 0;
			}
			result.success = frelements.add(goal->id.c_str(),(uint32_t*)goal->times.data(),values,(int)goal->states.size());
			if (result.success >=0)
			{
				mess << "Added " << result.success << " of the " << (int)goal->states.size() << " provided measurements to the state " << goal->id;
				result.message = mess.str(); 
			}else{
				mess << "A new state " <<  goal->id << " was added to the collection and filled with "  << (int)goal->states.size() << " measurements.";
				result.message = mess.str(); 
			}
			server->setSucceeded(result);
		}else{
			mess << "The length of the 'states' and 'times' arrays does not match.";
			result.message = mess.str(); 
			result.success = -2;
			server->setAborted(result);
		}
	}
	else if (goal->operation == "addvalues")
	{
		if (goal->times.size() == goal->values.size()){
			result.success = frelements.add(goal->id.c_str(),(uint32_t*)goal->times.data(),(float*)goal->values.data(),(int)goal->values.size());
			if (result.success >=0)
			{
				mess << "Added " << result.success << " of the " << (int)goal->values.size() << " provided measurements to the state " << goal->id;
				result.message = mess.str(); 
			}else{
				mess << "A new state " <<  goal->id << " was added to the collection and filled with "  << (int)goal->values.size() << " measurements.";
				result.message = mess.str(); 
			}
			server->setSucceeded(result);
		}else{
			mess << "The length of the 'values' and 'times' arrays does not match.";
			result.message = mess.str(); 
			result.success = -2;
			server->setAborted(result);
		}
	}
	else if (goal->operation == "evaluate")
	{
		if (goal->times.size() == goal->states.size()){
			float evaluations[goal->order+1];
			result.success = frelements.evaluate(goal->id.c_str(),(uint32_t*)goal->times.data(),(unsigned char*)goal->states.data(),(int)goal->times.size(),goal->order,evaluations);
			if (result.success >= 0)
			{
				mess << "Performed " <<  (goal->order+1) << " evaluations of the model "  << goal->id << " using " << (int)goal->times.size() << " ground truth values. The best performing model has order " << result.success;
				result.message = mess.str();
				result.errors.assign(evaluations,evaluations + goal->order+1);
				server->setSucceeded(result);
			}else{
				mess << "State ID " << goal->id << " does not exist.";
				result.message = mess.str();
				server->setAborted(result);
			}
		}else{
			mess << "The length of the 'states' and 'times' arrays does not match.";
			result.message = mess.str(); 
			result.success = -2;
			server->setAborted(result);
		}
	}
	else if (goal->operation == "predict")
	{
		float probs[goal->times.size()];
		result.success = frelements.estimate(goal->id.c_str(),(uint32_t*)goal->times.data(),probs,(int)goal->times.size(),goal->order);
		if (result.success >=0)
		{
			mess << "Performed " << (int)goal->times.size() << " predictions of the state " << goal->id;
			result.probabilities.assign(probs,probs + (int)goal->times.size());
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "State ID " << goal->id << " does not exist.";
			result.message = mess.str();
			server->setAborted(result);
		}
	}
	else if (goal->operation == "forecast")
	{
		float probabilities[goal->ids.size()];
		float probs[1];
		int32_t order = goal->order; 
		result.success = -1;
		if (goal->times.size() == 1){
			result.success = 0; 
			for (int i=0;i<goal->ids.size();i++)
			{
				if (goal->ids.size()==goal->orders.size()) order = goal->orders[i]; 
				if (frelements.estimate(goal->ids[i].c_str(),(uint32_t*)goal->times.data(),probs,1,order)==1){
					probabilities[i] = probs[0];
					result.success++;
				}else{
					probabilities[i] = -1;
				}				
			}
		}
		if (result.success >=0)
		{
			mess << "Predicted the probabilities of " << (int)goal->ids.size() << " states ";
			result.probabilities.assign(probabilities,probabilities + (int)goal->ids.size());
			result.entropies.clear();
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "The forecast service failed - the ''times'' argument did not have length equal to 1.";
			result.message = mess.str();
			server->setAborted(result);
		}
	}
	else if (goal->operation == "entropy")
	{
		float probs[goal->times.size()];
		result.success = frelements.estimateEntropy(goal->id.c_str(),(uint32_t*)goal->times.data(),probs,(int)goal->times.size(),goal->order);
		if (result.success >=0)
		{
			mess << "Performed " << (int)goal->times.size() << " entropy estimations of the state " << goal->id;
			result.entropies.assign(probs,probs + (int)goal->times.size());
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "State ID " << goal->id << " does not exist.";
			result.message = mess.str();
			server->setAborted(result);
		}
	}
	else if (goal->operation == "forecastEntropy")
	{
		float probabilities[goal->ids.size()];
		float probs[1];
		int32_t order = goal->order; 
		result.success = -1;
		if (goal->times.size() == 1){
			result.success = 0; 
			for (int i=0;i<goal->ids.size();i++)
			{
				if (goal->ids.size()==goal->orders.size()) order = goal->orders[i]; 
				if (frelements.estimateEntropy(goal->ids[i].c_str(),(uint32_t*)goal->times.data(),probs,1,order)==1){
					probabilities[i] = probs[0];
					result.success++;
				}else{
					probabilities[i] = -1;
				}				
			}
		}
		if (result.success >=0)
		{
			mess << "Predicted the entropies of " << (int)goal->ids.size() << " states ";
			result.entropies.assign(probabilities,probabilities + (int)goal->ids.size());
			result.probabilities.clear();
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "The forecast service failed - the ''times'' argument did not have length equal to 1.";
			result.message = mess.str();
			server->setAborted(result);
		}
	}
	else if (goal->operation == "remove")
	{
		result.success = frelements.remove(goal->id.c_str());
		if (result.success > 0)
		{
			mess << "State ID " << goal->id << " removed from the collection of " << result.success << " states.";
			result.message = mess.str();
			server->setSucceeded(result);
		}else{
			mess << "State ID " << goal->id << " does not exist.";
			result.message = mess.str();
			server->setAborted(result);
		}
	}
	else if (goal->operation == "view")
	{
		if (goal->order < NUM_PERIODICITIES){
			float amplitudes[goal->order+1];
			float periods[goal->order+1];
			float phases[goal->order+1];
			result.success = frelements.getModelParameters(goal->id.c_str(),periods,amplitudes,phases,goal->order);
			if (result.success == goal->order)
			{
				mess << "Returning " << goal->order << " FreMen model parameters of "  << goal->id << ". The probabilities field contains frequencies, the entropies field contains phase amplitudes and the errors field contains phase shifts. Infinite period corresponds to static probability.";
				result.probabilities.assign(periods,periods + (int)(goal->order+1));
				result.entropies.assign(amplitudes,amplitudes + (int)(goal->order+1));
				result.errors.assign(phases,phases + (int)(goal->order+1));

				result.message = mess.str();
				server->setSucceeded(result);
			}else{
				mess << "State ID " << goal->id << " does not exist.";
				result.message = mess.str();
				server->setAborted(result);
			}
		}else{
			mess << "The maximal fremen order is " << NUM_PERIODICITIES << ". You can't ask for more.";
			result.success = -1;
			server->setAborted(result);
		}
	}else if (goal->operation == "debug")
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
	float 		state[100000];
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
	
	for (int i = 0;i<len;i++)fprintf(file,"%.3f %.3f %.3f\n",state[i],probsA[i],probsB[i]);
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

	ros::Publisher starter_pub = n->advertise<std_msgs::Bool>("/fremenserver_start", 1, true);
	std_msgs::Bool msg;
	msg.data=true;
	starter_pub.publish(msg);

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}

	return 0;
}
