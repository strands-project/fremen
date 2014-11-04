#include "CEdgeStatistics.h"

/**
@author Tom Krajnik
*/

CEdgeStatistics::CEdgeStatistics(const char* iname)
{
 	length = 0;
	strcpy(name,iname);
}

CEdgeStatistics::~CEdgeStatistics()
{
}

bool CEdgeStatistics::addMeasurement(const char *date,const char *status,float duration)
{
	struct tm tm;
	time_t t;

	//converting text representation of the date to epoch
	int s;
	char datum[1000];
	sprintf(datum,"%s",date);
	if (strptime(datum,"%A, %b %d %Y, at %H:%M:%S hours", &tm) == NULL){
		//ROS_ERROR("Cannot convert date ""%s"" to epoch, trying to add year 2014 as a default one.",datum);
		sprintf(datum,"2014 %s",date);
		if (strptime(datum,"%Y %A, %b %d, at %H:%M:%S hours", &tm) == NULL){
			printf("Cannot convert date ""%s"" to epoch.",datum);
			t = 0;
		}else{
			t = mktime(&tm);
		}
	}else{
		t = mktime(&tm);
	}

	//conversion OK
	if (t != 0 && length< MAX_LENGTH){
		if (length == 0) firstTime = t;
		times[length] = t-firstTime;
		durations[length] = duration;
		results[length] = 0;
		if (strcmp(status,"success") == 0) results[length] = 1;
		length++;
	}
}

bool CEdgeStatistics::addMeasurement(int t,int status,float duration)
{
	if (length< MAX_LENGTH){
		if (length == 0) firstTime = t;
		times[length] = t-firstTime;
		durations[length] = duration;
		results[length] = status;
		length++;
	}
}

bool CEdgeStatistics::buildModel(int resultOrder,int timeOrder)
{
	//TODO remove - but why ?
	float resultsf[MAX_LENGTH];
	for (int i=0;i<length;i++) resultsf[i]=results[i];
	resultPredictor.build(times,resultsf,length,resultOrder);
	timePredictor.build(times,durations,length,timeOrder,results);
}

float CEdgeStatistics::predictResult(unsigned long int time)
{
	int predictTime = (int)(time-firstTime);
	return resultPredictor.estimate(predictTime);
}

float CEdgeStatistics::predictTime(unsigned long int time)
{
	int predictTime = (int)(time-firstTime);
	return timePredictor.estimate(predictTime);
}

bool CEdgeStatistics::load(FILE* file)
{
	int ret = 0;
	ret+=fread(name,1,100,file);
	ret+=fread(&firstTime,sizeof(long int),1,file);
	ret+=fread(&length,sizeof(int),1,file);
	ret+=fread(&durations,sizeof(float),length,file);
	ret+=fread(&results,sizeof(unsigned char),length,file);
	ret+=fread(&times,sizeof(int),length,file);
	//TODO fix this
	return true; 
}

bool CEdgeStatistics::save(FILE* file)
{
	fwrite(name,1,100,file);
	fwrite(&firstTime,sizeof(long int),1,file);
	fwrite(&length,sizeof(int),1,file);
	fwrite(&durations,sizeof(float),length,file);
	fwrite(&results,sizeof(unsigned char),length,file);
	fwrite(&times,sizeof(int),length,file);
}

bool CEdgeStatistics::print(int verbosityLevel)
{

}

