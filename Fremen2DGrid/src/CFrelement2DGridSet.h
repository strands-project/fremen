#ifndef CFRELEMENT2DGRIDSET_H
#define CFRELEMENT2DGRIDSET_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "CFrelement2DGrid.h"
#include <nav_msgs/OccupancyGrid.h>

#define MAX_LENGTH 1000

/**
@author Tom Krajnik
*/

using namespace std;

class CFrelement2DGridSet
{
	public:
		CFrelement2DGridSet();
		~CFrelement2DGridSet();

		/*add new measurements  
		  - if the name (ID) is new, then a new state is created the function 
		  - if not, the measurements are added to the state with the given ID 
		  - returns the the index of the set in the collection*/
		int add(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map);

		/*estimate probabilities of the given state for the given times - the probs array is an output
		  returns false if the state with the given ID is not present in the collection
		  otherwise returns true and fills the probs array with the calculated predictions*/
		int estimate(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map,int order);

		/*estimate entropies of the given state for the given times - the entropy array is an output
		  returns false if the state with the given ID is not present in the collection
		  otherwise returns true and fills the probs array with the calculated predictions*/
//		int estimateEntropy(const char *name,uint32_t times[],float entropy[],int length,int order);

		/*evaluate the prediction/estimation for the given states and times
		  returns -1 if the state with the given ID is not present in the collection
		  otherwise returns the best performing model order and the errors in the eval array*/
//		int evaluate(const char *name,uint32_t times[],unsigned char probs[],int length,int order,float eval[]);

		/*remove states from the collection
		  return the number of remaining states*/
//		int remove(const char *name);

		/*remove states from the collection
		  returns false if the state with the given ID is not present in the collection*/
//		bool update(const char* name,int order);


		CFrelement2DGrid* active;
	private:
		bool find(const char *name);
		CFrelement2DGrid* grids[MAX_LENGTH];
		int numGrids;
		int activeIndex;
};

#endif //CEDGESTATISTICS_H

