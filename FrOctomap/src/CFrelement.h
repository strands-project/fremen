#ifndef CFRELEMENT_H
#define CFRELEMENT_H

#include <iostream>
#include <vector>
#include <complex>	
#include <algorithm> 
#include <iterator> 
#include <complex>	// goes along with fftw
#include <fftw3.h>
#include <string.h>
#include "CTimer.h"
#include "CFFTPlan.h"
	
/**
@author Tom Krajnik
*/

using namespace std;

typedef struct
{
	float amplitude;
	float phase;
	unsigned int frequency;
}SFrelement;

class CFrelement
{
public:
  CFrelement();
  ~CFrelement();

  float estimate(float* signal,CFFTPlan *plan,float anomalyThreshold = 1.0);
  float reconstruct(unsigned char* signal,CFFTPlan *plan,bool evaluate = false);

  /*state estimation: retrieves the state*/
  float estimate(int timeStamp);
  float fineEstimate(float timeStamp);

  /*state estimation: retrieves the state*/
  unsigned char retrieve(int timeStamp);

  /*adds a single measurement*/
  void add(unsigned char value);

  /*gets length of the stored signal*/
 unsigned int getLength();

  void build(unsigned char* signal,int signalLength,CFFTPlan *plan);

  void print(bool verbose=true);

  int save(FILE* file,bool lossy = false,int forceOrder = -1);
  int load(FILE* file);
  int save(char* name,bool lossy = false,int forceOrder = -1);
  int load(char* name);

  /*changes the model order*/
  float update(int modelOrder,CFFTPlan *plan,bool evaluate = false);

  /*evaluates the model precision*/
  int evaluatePrecision(CFFTPlan *plan,float errors[],int maxOrder);

  double *signal;

//private:
	SFrelement *frelements;
	unsigned int *outlierSet;
	unsigned int outliers;
	unsigned char order;
	float gain;
	unsigned int signalLength;
};

#endif
