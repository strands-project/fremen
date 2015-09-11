#ifndef CFRELEMENT2DGRID_H
#define CFRELEMENT2DGRID_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "CFrelement.h"

/**
@author Tom Krajnik
*/

using namespace std;

class CFrelement2DGrid
{
	public:
		CFrelement2DGrid(const char* name);
		~CFrelement2DGrid();

		/*add new measurements  
		  - adds a new 2d grid, if it's the first grid added, then its params are stored and the following grids will be fitted
		  - states are [-1..100], where -1 is unknown, 0 is free and 100 is occupied
		  - returns the number of stored grids*/
		int add(uint32_t time,int8_t states[],int width,int height,float originX,float originY,float resolution);

		/*returns the occupancy probabilities for the given time with a given order 
		  returns the number of predicted cells*/
		int estimate(uint32_t time,int8_t states[],int order);

		/*estimate entropies of the given state for the given times - the entropy array is an output
		  returns false if the state with the given ID is not present in the collection
		  otherwise returns true and fills the probs array with the calculated predictions*/
		int estimateEntropy(uint32_t time,float entropy[],int order);

		/*evaluate the prediction/estimation for the given states and times
		  returns -1 if the state with the given ID is not present in the collection
		  otherwise returns the best performing model order and the errors in the eval array*/
		int evaluate(const char *name,uint32_t times[],unsigned char probs[],int length,int order,float eval[]);

		/*print grid info*/
		bool print(int order);

		/*load the 2D grid from a file*/
		int load(const char* file);

		/*save the 2D grid to a file*/
		int save(const char* file);

		/*idecko voe*/
		char id[255];

	private:
		CFrelement** frelementArray;
		int numFrelements;
		int width;
		int height;
		float resolution;
		float originX;
		float originY;
};

#endif 

