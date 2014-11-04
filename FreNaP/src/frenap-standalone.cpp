#include <stdlib.h>
#include "CFrelement.h"
#define MAX_LENGTH 1000000

using namespace std;
int times[MAX_LENGTH]; 
float signal[MAX_LENGTH]; 
int signalLength = 0;
CFrelement frelement;
long int firstTime = 0;

int debg =0;
bool debug = false;

int numEdges = 0;
char mapName[1000];

int loadSignal(char* name)
{
	FILE* file = fopen(name,"r");
	if (file == NULL) {
		fprintf(stderr,"Could not find file %s\n",name);
		return -1;
	}
	signalLength = 0;
	long int timeO;
	float signalO;
	int ret = 0;
	while (feof(file)==0)
	{
		ret = fscanf(file,"%ld %f\n",&timeO,&signalO);
		if (signalLength == 0) firstTime = timeO;
		times[signalLength] = (int)(timeO-firstTime);
		signal[signalLength] = signalO;		
		signalLength++;
	}
	fclose(file);
}

int main(int argc,char* argv[])
{
	loadSignal(argv[1]);
	frelement.build(times,signal,signalLength,atoi(argv[2]),NULL);
	for (int i = times[0];i<times[signalLength-1];i+=atoi(argv[3])) printf("%ld %f\n",i+firstTime,frelement.estimate(i)); 
	return 0;
}
