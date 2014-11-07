#ifndef CFRELEMENT_H
#define CFRELEMENT_H

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <string.h>
#include "CTimer.h"

#define MAX_ADAPTIVE_ORDER 4
	
/**
@author Tom Krajnik
*/

using namespace std;

typedef struct
{
	float amplitude;
	float phase;
	float frequency;
}SFrelement;

class CFrelement
{
	public:
		CFrelement();
		~CFrelement();

		void add(unsigned long int time,unsigned char data);
		void build(int* times,float* signal,int length,int orderi,unsigned char* status = NULL);
		float evaluate(int* times,float* signal,int length,int orderi,unsigned char* status = NULL);
		unsigned char retrieve(unsigned long int timeStamp);
		void update(int modelOrder);
		float estimate(int time);
		void print(bool verbose=true);

		int save(FILE* file,bool lossy = false);
		int load(FILE* file);
		int save(char* name,bool lossy = false);
		int load(char* name);

		double *signal;

		SFrelement *frelements;
		unsigned long int *outlierSet;
		unsigned int outliers;
		unsigned char order;
		float gain;
		unsigned long int timePeriod;
		unsigned long lastTime;
		int signalLength;
};

#endif
