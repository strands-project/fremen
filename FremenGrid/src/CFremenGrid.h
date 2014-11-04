#ifndef CFREMENGRID_H
#define CFREMENGRID_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "CTimer.h" 

/**
@author Tom Krajnik
*/

using namespace std;

class CFremenGrid
{
	public:
		CFremenGrid(float originX,float originY,float originZ,int dimX,int dimY,int dimZ,float cellSize);
		~CFremenGrid();

		/*state estimation: estimates the state of the i-th element*/
		float estimate(float x,float y,float z,float timeStamp);
		float getInformation(float x,float y,float z,float phiRange,float psiRange,float range,float timeStamp);

		/*changes the model order*/
		void print(bool verbose);
		void save(const char*name,bool lossy = false,int forceOrder = -1);
		bool load(const char*name);

		float estimate(unsigned int index,float timeStamp);
		void incorporate(float *x,float *y,float *z,float *d,int size);
		int getIndex(float x,float y,float z);

		//center of the first cell
		float oX,oY,oZ;		
		//size of the grid cells
		float resolution;
		//grid dimensions
		int xDim,yDim,zDim;
		float* probs;	
		char *aux;
		int numCells;
		bool debug;

		float lastPhiRange,lastPsiRange,lastRange;
		int *raycasters;
		int numRaycasters;
};

#endif
