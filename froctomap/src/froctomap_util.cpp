#include <iostream>
#include <fstream>	
#include <cstdlib>	
#include "CFremenGrid.h"
#include "CTimer.h"

using namespace std;

int generateMaps(const char* name,const char *output,int startTime,int endTime,int interval)
{
	/*load the fremen grid*/
	CFremenGrid *grid = new CFremenGrid(1,1,1);
	grid->load(name);

	/*prepare output*/
	long int numTimes = (endTime-startTime)/interval;
	long int gridSize = grid->numCells;
	unsigned char* states =  (unsigned char*) malloc(gridSize*numTimes);

	if (states == NULL)
	{
		printf("Could not allocate %ld MB of memory.\n",gridSize*numTimes/1024/1024);
		return -1;
	}

	/*perform retrieval*/
	long int cnt = 0;
	for (long int i = 0;i<numTimes;i++){
		for (long int s = 0;s<gridSize;s++) states[cnt++] = grid->retrieve(s, i*interval+startTime);
		printf("3D Grid %ld of %ld generated !\n",i,numTimes);
	}

	/*save it*/
	FILE* file = fopen(output,"w+");
	fwrite(states,sizeof(unsigned char),gridSize*numTimes,file);
	fclose(file);
	printf("File %s saved.\n",output);
	delete grid;
	free(states);
}

int main(int argc,char *argv[])
{
	unsigned char *signal = (unsigned char*)malloc(10000000);
	unsigned char *reconstructed = (unsigned char*)malloc(10000000);
	int signalLength = 0;

	CTimer timer;
	timer.start();
	CFremenGrid grid(1,1,1);
	if (strcmp(argv[1],"reconstruct")==0)
	{
		generateMaps(argv[2],argv[3],atoi(argv[4]),atoi(argv[5]),atoi(argv[6]));
	}
	else if (strcmp(argv[1],"retrieve")==0){
		grid.load(argv[2]);
		grid.reconstruct(atoi(argv[1]),reconstructed);
		for (int i=0;i<grid.signalLength;i++)printf("%i ",reconstructed[i]);
	}
	else if (strcmp(argv[1],"overview")==0)
	{
		grid.load(argv[2]);
		grid.print(false);
	}
	else if (strcmp(argv[1],"print")==0)
	{
		grid.load(argv[2]);
		grid.print(true);
	}
	else if (strcmp(argv[1],"updateOne")==0)
	{
		grid.load(argv[2]);
		grid.updateOne(atoi(argv[2]),atoi(argv[3]));
		grid.save(argv[4],false);
	}
	else if (strcmp(argv[1],"update")==0)
	{
		grid.load(argv[2]);
		grid.update(atoi(argv[3]),grid.signalLength);
		grid.save(argv[4],false);
	}
	else
	{
		fprintf(stderr,"fremen reconstruct input_grid dump start_time end_time interval\n");
		fprintf(stderr,"fremen updateOne input_grid cell_index fremen_order output_grid\n");
		fprintf(stderr,"fremen updateAll input_grid fremen_order output_grid\n");
		fprintf(stderr,"fremen retrieveOne input_grid cell_index output_grid\n");
		fprintf(stderr,"fremen overview input_grid\n");
		fprintf(stderr,"fremen print input_grid\n");
		return -1;
	}
	/*cout << "Model update time " << timer.getTime()/atoi(argv[2])/1000 << " ms." << endl;

	  grid.reconstruct(0,reconstructed);

	  int err = 0;
	  int pos = 0;
	  for (int i=0;i<signalLength;i++) err+=abs((int)signal[i]-(int)reconstructed[i]);
	  for (int i=0;i<signalLength;i++) pos+=(int)signal[i];
	  cout << "Perfect: " << err << " " << pos << endl;*/
	//	free(signal);
	//	free(reconstructed);
	return 0;
}
