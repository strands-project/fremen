#include <iostream>
#include <fstream>	
#include <cstdlib>	
#include "CFremenGrid.h"
#include "CTimer.h"

using namespace std;

int main(int argc,char *argv[])
{
	unsigned char *signal = (unsigned char*)malloc(10000000);
	unsigned char *reconstructed = (unsigned char*)malloc(10000000);
	int signalLength = 0;

	CTimer timer;
	timer.start();
	CFremenGrid grid(1,1,1);
	if (strcmp(argv[1],"retrieve")==0){
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
