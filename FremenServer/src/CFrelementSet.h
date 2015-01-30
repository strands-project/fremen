#ifndef CFRELEMENTSET_H
#define CFRELEMENTSET_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "CFrelement.h"

#define MAX_LENGTH 10000

/**
@author Tom Krajnik
*/

using namespace std;

class CFrelementSet
{
	public:
		CFrelementSet();
		~CFrelementSet();

		/*add new measurements  
		  - if the name (ID) is new, then a new state is created the function returns 'false'
		  - if not, the measurements are added to the state with the given ID and 'true' is returned*/
		int add(const char *name,uint32_t times[],unsigned char states[],int length);

		/*estimate probabilities of the given state for the given times - the probs array is an output
		  returns false if the state with the given ID is not present in the collection
		  otherwise returns true and fills the probs array with the calculated predictions*/
		int estimate(const char *name,uint32_t times[],float probs[],int length,int order);

		/*estimate entropies of the given state for the given times - the entropy array is an output
		  returns false if the state with the given ID is not present in the collection
		  otherwise returns true and fills the probs array with the calculated predictions*/
		int estimateEntropy(const char *name,uint32_t times[],float entropy[],int length,int order);

		/*evaluate the prediction/estimation for the given states and times
		  returns -1 if the state with the given ID is not present in the collection
		  otherwise returns the best performing model order and the errors in the eval array*/
		int evaluate(const char *name,uint32_t times[],unsigned char probs[],int length,int order,float eval[]);

		/*remove states from the collection
		  return the number of remaining states*/
		int remove(const char *name);

		/*remove states from the collection
		  returns false if the state with the given ID is not present in the collection*/
		bool update(const char* name,int order);

		/*print model info*/
		bool print(int verbosityLevel);

		/*load the model from a file*/
		bool load(FILE* file);

		/*save the model to a file*/
		bool save(FILE* file);

	private:
		bool find(const char *name);
		CFrelement* frelements[MAX_LENGTH];
		CFrelement* active;
		int numFrelements;
		int activeIndex;
};

#endif //CEDGESTATISTICS_H

