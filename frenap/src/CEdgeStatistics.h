#ifndef CEDGESTATISTICS_H
#define CEDGESTATISTICS_H

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "CFrelement.h"

#define MAX_LENGTH 10000

/**
@author Tom Krajnik
*/

using namespace std;

class CEdgeStatistics
{
public:
	CEdgeStatistics(const char* name);
	~CEdgeStatistics();
	bool addMeasurement(const char *date,const char* status,float duration);
	bool addMeasurement(int time,int status,float duration);
	bool print(int verbosityLevel);
	bool load(FILE* file);
	bool save(FILE* file);
	bool buildModel(int resultOrder,int timeOrder);
	float predictResult(unsigned long int time);
	float predictTime(unsigned long int time);

	char name[100];
	long int firstTime;
	float durations[MAX_LENGTH];
	unsigned char results[MAX_LENGTH];
	int times[MAX_LENGTH];
	int length;

	CFrelement resultPredictor;
	CFrelement timePredictor;
};

#endif //CEDGESTATISTICS_H

