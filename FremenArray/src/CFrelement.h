#ifndef CFRELEMENT_H
#define CFRELEMENT_H

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <stdint.h>

#define NUM_PERIODICITIES 100
#define FREMEN_AMPLITUDE_THRESHOLD 0.0
	
/**
@author Tom Krajnik
*/
typedef struct{
	float realStates;
	float imagStates;
	float realBalance;
	float imagBalance;
	float amplitude;
	float phase;
	float period;	
}SFrelement;

using namespace std;

class CFrelement
{
	public:
		CFrelement();
		~CFrelement();

		//adds a serie of measurements to the data
		int add(uint32_t times[],float states[],int length);

		//estimates the probability for the given times 
		int estimate(uint32_t times[],float probs[],int length,int order);

		//estimates the state's entropy for the given times 
		int estimateEntropy(uint32_t times[],float entropy[],int length,int order);

		//evaluates the error of the predictions for the given times and measurements
		int evaluate(uint32_t times[], float signal[],int length,int order,float evals[]);
	
		void update(int modelOrder);
		void print(int order);

		int save(FILE* file,bool lossy = false);
		int load(FILE* file);
		int save(char* name,bool lossy = false);
		int load(char* name);
		
		float gain;
		SFrelement frelements[NUM_PERIODICITIES];
		int measurements;
		int64_t firstTime;
		int64_t  lastTime;
};

#endif
