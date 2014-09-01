#ifndef CFREMENGRID_H
#define CFREMENGRID_H

#include "CFrelement.h"
	
/**
@author Tom Krajnik
*/

typedef struct{
	float all;
	float nonempty;
	float dynamic;
}SGridErrors;

using namespace std;

class CFremenGrid
{
	public:
		CFremenGrid(int dimX,int dimY,int dimZ);
		~CFremenGrid();

		/*state estimation: estimates the state of the i-th element*/
		float estimate(unsigned int index,unsigned int timeStamp);
		float fineEstimate(unsigned int index,float timeStamp);

		/*state estimation: retrieves the state of the i-th element*/
		unsigned char retrieve(unsigned int index,unsigned int timeStamp);

		/*fills with values*/
		void fill(unsigned char values[],unsigned int timeStamp);

		void add(unsigned int index,unsigned char value);

		/*clears all*/
		void clear();

		/*sets the octomap's origin*/
		void setPose(float x, float y);
		void setName(const char* n);

		/*changes the model order*/
		SGridErrors update(int order,int signalLengthi,bool evaluate = false);
		void updateOne(int cellIndex, int order);
		void reconstruct(int number,unsigned char *reconstructed);
		void print(bool verbose);
		void reconstruct();
		void save(const char*name,bool lossy = false,int forceOrder = -1);
		bool load(const char*name);

		/*for evaluation purposes*/
		int evaluatePrecision(SGridErrors e[],int maxOrder);

		//private:
		double *signal;
		char name[100];
		CFrelement **cellArray;
		int numCells;
		int xDim,yDim,zDim;
		int order;
		unsigned int signalLength;
		CFFTPlan *plan;
		float positionX,positionY;
};

#endif
