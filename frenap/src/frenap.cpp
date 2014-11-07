#include <stdlib.h>
#include "ros/ros.h"
#include <mongodb_store/message_store.h>
#include "CEdgeStatistics.h"
#include <boost/foreach.hpp>
#include <strands_navigation_msgs/NavStatistics.h>
#include <actionlib/server/simple_action_server.h>
#include <frenap/TopologicalPredictionAction.h>

#define MAX_EDGES 10000

const mongo::BSONObj EMPTY_BSON_OBJ;

using namespace mongodb_store;
using namespace std;

ros::NodeHandle *n;
bool debug = false;

int numEdges = 0;
CEdgeStatistics *edgeArray[MAX_EDGES];
char mapName[1000];
typedef actionlib::SimpleActionServer<frenap::TopologicalPredictionAction> Server;
Server* server;
frenap::TopologicalPredictionResult result;
frenap::TopologicalPredictionFeedback feedback;

void retrieveData(const char* mapName)
{
	for (int i = 0;i<numEdges;i++) free(edgeArray[numEdges]);
	numEdges = 0;
	MessageStoreProxy messageStore(*n,"message_store");
//	string name(filename);
	vector< boost::shared_ptr<strands_navigation_msgs::NavStatistics> > results;
        mongo::BSONObj meta_query = BSON( "type" << "Topological Navigation Stat");
	mongo::BSONObj data_query = BSON( "topological_map" << mapName);
	feedback.status = "Waiting for Mongo to process the query";
	server->publishFeedback(feedback);
	if (strcmp(mapName,"all")==0){
		messageStore.query<strands_navigation_msgs::NavStatistics>(results, EMPTY_BSON_OBJ, meta_query);
	}else{
		messageStore.query<strands_navigation_msgs::NavStatistics>(results, data_query, meta_query);
	}
	feedback.status = "Building edge information";
	server->publishFeedback(feedback);
	BOOST_FOREACH( boost::shared_ptr<strands_navigation_msgs::NavStatistics> p, results)
	{
		int index = -1;
		char edgename[100];
		if (strcmp(mapName,"all")==0) printf("MAP: %s\n",p->topological_map.c_str());
		if (strcmp(p->topological_map.c_str(),mapName)==0){
			sprintf(edgename,"%s->%s",p->origin.c_str(),p->target.c_str());
			for (int i = 0;i<numEdges && index == -1;i++)
			{	
				if (strcmp(edgename,edgeArray[i]->name)==0) index = i;
			}
			if (index == -1){
				edgeArray[numEdges] = new CEdgeStatistics(edgename);
				index = numEdges++;
			}
			edgeArray[index]->addMeasurement(p->date_started.c_str(),p->status.c_str(),p->operation_time);
		}
	}
	if (debug) printf("\n");
	for (int i = 0;i<numEdges && debug;i++){
		 printf("Integral: %s - %i:",edgeArray[i]->name,edgeArray[i]->length);
		 for (int j = 0;j<edgeArray[i]->length;j++)printf("%i,",edgeArray[i]->results[j]);
		 printf("\n");
	}
}

/*testing purposes*/
int saveData(char* filename)
{
	FILE* file=fopen(filename,"w");
	fwrite(&numEdges,sizeof(int),1,file);
	for (int i = 0;i<numEdges;i++)edgeArray[i]->save(file);
	fclose(file);
}

/*testing purposes*/
int loadData(const char* filename)
{
	FILE* file=fopen(filename,"r");
	if (file == NULL) return -1;
	int ret = fread(&numEdges,sizeof(int),1,file);
	for (int i = 0;i<numEdges;i++){
		edgeArray[i] = new CEdgeStatistics("");
		edgeArray[i]->load(file);
	}
	fclose(file);
	return 0;
}

int printEdges(const char* name)
{
	bool printstuff = false;
	if (strcmp(name,"all")==0) printstuff = true;
	for (int i = 0;i<numEdges;i++){
			printf("Integral: %s - %i:",edgeArray[i]->name,edgeArray[i]->length);
			for (int j = 0;j<edgeArray[i]->length && printstuff ;j++)printf("%i,",edgeArray[i]->results[j]);
			printf("\n");
	}
}

int debugPrint(const char* name)
{
	int index = -1;
	//if (strcmp(name,"all")==0) printstuff = true;
	for (int i = 0;i<numEdges;i++){
		if (strcmp(edgeArray[i]->name,name)==0) index = i;
	}
/*	long int timeOffset = 600*3600;
	index = numEdges;	
	edgeArray[index] = new CEdgeStatistics("");
	for (int i = 0;i<24*7;i++){
		edgeArray[index]->addMeasurement(i*3600+timeOffset,(float)((i%24)>20),0);
		if (i==24) i=4*24;
	}*/
	if (index != -1){
		edgeArray[index]->resultPredictor.print();
		for (int i = 0;i<edgeArray[index]->length;i++){
			printf("Data %ld %i %.1f \n",edgeArray[index]->times[i]+edgeArray[index]->firstTime,edgeArray[index]->results[i],edgeArray[index]->durations[i]);
		}
		for (int i = edgeArray[index]->firstTime;i<edgeArray[index]->times[edgeArray[index]->length-1]+edgeArray[index]->firstTime;i+=3600){
			printf("Prediction: %i %.3f %.3f\n",i,edgeArray[index]->predictResult(i),edgeArray[index]->predictTime(i));
		}
		float probError = 0;
		float realError = 0;
		float durError = 0;
		float durNum = 0;
		for (int i = 0;i<edgeArray[index]->length;i++)
		{
			float errorEstimate = edgeArray[index]->predictResult(edgeArray[index]->times[i]+edgeArray[index]->firstTime);
			float timeEstimate = edgeArray[index]->predictTime(edgeArray[index]->times[i]+edgeArray[index]->firstTime);
			probError += fabs(errorEstimate-edgeArray[index]->results[i]);
			realError += fabs((errorEstimate>=0.5)-edgeArray[index]->results[i]);
			if(edgeArray[index]->results[i]==1){
				 durError += fabs(timeEstimate-edgeArray[index]->durations[i]);
				 durNum++;
			}
		}
		printf("Model order %i %i, probabilistic error %.3f, absolute error %.3f, duration error %.3f.\n",edgeArray[index]->resultPredictor.order,edgeArray[index]->timePredictor.order,probError/edgeArray[index]->length,realError/edgeArray[index]->length,durError/durNum);
	}
}

int getTimeline(const char* name,long int startTime,long int endTime,int step)
{
	int index = -1;
	for (int i = 0;i<numEdges;i++){
		if (strcmp(edgeArray[i]->name,name)==0) index = i;
	}
	if (index != -1)
	{
		result.edgeName.clear();
		result.probability.clear();
		result.duration.clear();
		result.edgeName.push_back(edgeArray[index]->name);
		for (int i = startTime;i<endTime;i+=step)
		{
			result.probability.push_back(edgeArray[index]->predictResult(i));
			result.duration.push_back(edgeArray[index]->predictTime(i));
		}
		result.message = "Prediction provided.";
	}else{
		result.message = "Edge of that name does not exist.";
	}
	return index; 
}

void evaluateModels()
{
	result.edgeName.clear();
	result.probability.clear();
	result.duration.clear();
	for (int j = 0;j<numEdges;j++)
	{
		float probError = 0;
		float realError = 0;
		float durError = 0;
		float durNum = 0;
		int length=edgeArray[j]->length;
		for (int i = 0;i<length;i++)
		{
			float errorEstimate = edgeArray[j]->predictResult(edgeArray[j]->times[i]+edgeArray[j]->firstTime);
			float timeEstimate = edgeArray[j]->predictTime(edgeArray[j]->times[i]+edgeArray[j]->firstTime);
			probError += fabs(errorEstimate-edgeArray[j]->results[i]);
			realError += fabs((errorEstimate>=0.5)-edgeArray[j]->results[i]);
			if(edgeArray[j]->results[i]==1){
				durError += fabs(timeEstimate-edgeArray[j]->durations[i]);
				durNum++;
			}
		}
		if (durNum == 0) durNum = 1; 
		result.edgeName.push_back(edgeArray[j]->name);
		result.probability.push_back(realError/length);
		result.duration.push_back(durError/durNum);
		if (debug) printf("%s: orders %i %i, probabilistic error %.3f, absolute error %.3f, duration estimation error %.3f.\n",edgeArray[j]->name,edgeArray[j]->resultPredictor.order,edgeArray[j]->timePredictor.order,probError/length,realError/length,durError/durNum);
	}
}


void actionServerCallback(const frenap::TopologicalPredictionGoalConstPtr& goal, Server* as)
{
	char message[1000];
	if (debug) printf("Command received %s %s\n",goal->action.c_str(),goal->mapName.c_str());
	if (goal->action == "build")
	{
		retrieveData(goal->mapName.c_str());
		/*if (loadData(goal->mapName.c_str())!=0)
		{
			result.message = "Could not load file";
			server->setAborted(result);
			return;
		}*/
		if (debug) printEdges("all");
		feedback.status = "Building edge FreMEn models.";
		server->publishFeedback(feedback);
		for (int i = 0;i<numEdges;i++)edgeArray[i]->buildModel(goal->resultOrder,goal->durationOrder);
		evaluateModels();
		feedback.status = "FREMEN model build and evaluated.";
		server->publishFeedback(feedback);
		sprintf(message,"Build respective models of %i edges",numEdges);
		result.success = true;
		result.message = message;
		server->setSucceeded(result);
	}	
	else if (goal->action == "predict")
	{
		result.edgeName.clear();
		result.probability.clear();
		result.duration.clear();
		for (int i = 0;i<numEdges;i++){
			  printf("%s %.2f %.2f\n",edgeArray[i]->name,edgeArray[i]->predictResult(goal->predictionTime),edgeArray[i]->predictTime(goal->predictionTime));
			  result.edgeName.push_back(edgeArray[i]->name);
			  result.probability.push_back(edgeArray[i]->predictResult(goal->predictionTime));
			  result.duration.push_back(edgeArray[i]->predictTime(goal->predictionTime));
		}
		result.success = true;	
		result.message = "Prediction performed";
		server->setSucceeded(result);
	}
	else if (goal->action == "debug")
	{
		debugPrint(goal->mapName.c_str());
		result.success = true;	
		result.message = "Debug printed";
		server->setSucceeded(result);
	}
	else if (goal->action == "timeline")
	{
		if (getTimeline(goal->mapName.c_str(),goal->startTime,goal->endTime,goal->predictionTime)) server->setSucceeded(result); else server->setAborted(result);
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
	ros::init(argc, argv, "frenap");
	n = new ros::NodeHandle();
	server = new Server(*n, "/FreNaP", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	//retrieveData();
	//loadData(argv[1]);
	//printEdges(argv[2]);
	//debugPrint(argv[2]);

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
	return 0;
}
