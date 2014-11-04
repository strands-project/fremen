#include "CFrelement.h"

using namespace std;
static bool debug = false; 

bool fremenSort(SFrelement i,SFrelement j) 
{ 
	return (i.amplitude>j.amplitude); 
}

CFrelement::CFrelement()
{
	timePeriod = 7*24*3600;  //one week by default
	signalLength = gain = outliers = order = 0;
	frelements = NULL;
	outlierSet = NULL;
}

CFrelement::~CFrelement()
{
	free(frelements);
	free(outlierSet);
}

// adds new measurement at time a
void CFrelement::add(unsigned long int time,unsigned char value)
{
	if (((estimate(signalLength) > 0.5)^((outliers%2)==1))!=value)
	{
		unsigned long int* outlierSetTmp = (unsigned long int*)realloc(outlierSet,(outliers+1)*(sizeof(unsigned long int)));
		if (outlierSetTmp==NULL) fprintf(stderr,"Failed to reallocate the outlier set!\n"); 
		outlierSet = outlierSetTmp;
		outlierSet[outliers++] = signalLength;
	}
	signalLength++;
	return; 
}

float CFrelement::evaluate(int* times,float* signal,int length,int orderi,unsigned char* status)
{
	float error = 0;
	unsigned char mul = 1;
	int samples = 0; 
	int myOrder = order;
	order = orderi;
	for (int i = 0;i<length;i++)
	{	
		if (status != NULL) mul = status[i];
		error += mul*fabs(estimate(times[i])-signal[i]);
		samples+= mul;
	}
	order = myOrder;
	if (samples == 0) return 0;
	return error/samples;
}


void CFrelement::build(int* times,float* signal,int length,int orderi,unsigned char* status)
{
	if (orderi < 0){
		build(times,signal,length,MAX_ADAPTIVE_ORDER,status);
		if (debug) printf("Order:");
		float minError = 100000000;
		int minIndex = 0;
		for (int i = 0;i<MAX_ADAPTIVE_ORDER;i++)
		{
			float error = evaluate(times,signal,length,i,status);
			if (error < minError){
				minError = error;
				minIndex = i;
			}
			if (debug) printf(" %f ",error);
		}
		if (debug) printf(" - %i \n",minIndex);
		build(times,signal,length,minIndex,status);
	}else{
		int numFrequencies = timePeriod/2/3600;
		float frequencies[numFrequencies];
		for (int i = 0;i<numFrequencies;i++) frequencies[i] = (float)timePeriod/(i+1);
		float real[numFrequencies];
		float imag[numFrequencies];
		float amplitude[numFrequencies];
		memset(real,0,numFrequencies*sizeof(float));
		memset(imag,0,numFrequencies*sizeof(float));
		signalLength = 0;
		if (order > 0) free(frelements);
		order = orderi;
		frelements = (SFrelement*) malloc(order*sizeof(SFrelement));
		gain = 0;

		/*naive calculation of non uniform frequency transform*/
		if (status == NULL){
			signalLength = length;
			for (int j = 0;j<length;j++) gain += signal[j]; 
		}else{
			signalLength = 0;
			for (int j = 0;j<length;j++){
				gain += status[j]*signal[j]; 
				signalLength+=status[j];
			}
		}

		/*not enough data to support the build the models*/
		if (signalLength == 0)
		{
			gain = 0;
			order = 0;
			return;
		}

		/*at least some data points to build a sufficient model*/
		gain/=signalLength;
		float balance = gain;
		for (int j = 0;j<length;j++)
		{
			if (status == NULL || status[j]==1){
				for (int i = 0;i<numFrequencies;i++)
				{
					real[i] += (signal[j]-balance)*cos(2*M_PI*(float)times[j]/frequencies[i]);
					imag[i] += (signal[j]-balance)*sin(2*M_PI*(float)times[j]/frequencies[i]);
				}
			}
		} 

		/*selection of most prominent frequencies - the number of frequencies defines the fremen system order*/
		SFrelement *tmpFrelements = (SFrelement *) malloc(numFrequencies*sizeof(SFrelement));
		for (int i = 0;i<numFrequencies;i++)
		{
			tmpFrelements[i].amplitude = real[i]*real[i]+imag[i]*imag[i];
			tmpFrelements[i].frequency = i;
		}
		int index;
		partial_sort(tmpFrelements,tmpFrelements+order,tmpFrelements+numFrequencies,fremenSort);
		for(int i=0;i<order;i++){
			index = (int)tmpFrelements[i].frequency;
			frelements[i].amplitude = sqrt(tmpFrelements[i].amplitude)/signalLength;
			frelements[i].phase = atan2(imag[index],real[index]);
			frelements[i].frequency = frequencies[index];
		}
		free(tmpFrelements);

		/*debugging information*/
		if (debug){
			printf("Selected frequencies: %i \n",order);
			for (int i = 0;i<order;i++) printf("%.2f %.3f %.3f\n",frelements[i].frequency/3600.0,frelements[i].amplitude,frelements[i].phase);
		}
	}	
	/*calculation of the outlier set*/
	/*int j=0;
	unsigned char flip = 0;
	for (int i = 0;i<signalLength;i++)
	{
		if (signal[i] != retrieve(times[i])^flip)
		{
			flip = 1-flip;
			unsigned long int* outlierSetTmp = (unsigned long int*)realloc(outlierSet,(outliers+1)*(sizeof(int)));
			if (outlierSetTmp == NULL) fprintf(stderr,"Failed to reallocate the outlier set!\n"); 
			outlierSet = outlierSetTmp;
			outlierSet[outliers++] = times[i];
		}
	}*/
	return;
}

/*not implemented in the moment*/
void CFrelement::update(int modelOrder)
{
/*	if (order == 0 && outliers == 0){
	}else{
		unsigned char *reconstructed = (unsigned char*)malloc(signalLength*sizeof(unsigned char));
		reconstruct(reconstructed,plan);
		order = modelOrder;
		outliers = 0;
		free(frelements);
		frelements = (SFrelement*) malloc(order*sizeof(SFrelement));
		if (frelements == NULL) fprintf(stderr,"Failed to reallocate spectral components!\n");
		build(reconstructed,signalLength,plan);
		free(reconstructed);
	}*/
}

/*text representation of the fremen model*/
void CFrelement::print(bool verbose)
{
	int errs = 0;
	for (int i=0;i<outliers/2;i++) errs+=(outlierSet[2*i+1]-outlierSet[2*i]);
	if (outliers%2 == 1) errs+=signalLength-outlierSet[outliers-1];
	std::cout << "model order " << (int)order << " prior: " << gain << " error: " << ((float)errs/signalLength) << " size: " << signalLength << " ";
	if (order > 0) std::cout  << endl;
	if (verbose){
		float ampl = gain;
		for (int i = 0;i<order;i++){
			std::cout << "frelement " << i << " " << ampl << " " << frelements[i].phase << " " << frelements[i].frequency << " " << endl;
			ampl+=frelements[i].amplitude*frelements[i].amplitude;
		}
	}
	std::cout << "outlier set size " << outliers << ":";
	if (verbose){
		for (int i = 0;i<outliers;i++) std::cout << " " << outlierSet[i];
	}
	std::cout << endl; 
}

float CFrelement::estimate(int time)
{
	float estimate = gain;
	for (int i = 0;i<order;i++){
		estimate+=2*frelements[i].amplitude*cos(time/frelements[i].frequency*2*M_PI-frelements[i].phase);
	}
//	if (estimate > 1.0) return 1.0;
//	if (estimate < 0.0) return 0.0;
	return estimate;
}

/*retrieves a boolean*/
unsigned char CFrelement::retrieve(unsigned long int timeStamp)
{
	int i = 0;
	for (i= 0;i<outliers;i++){
		if (timeStamp < outlierSet[i]) break;
	}
	return (estimate(timeStamp) >= 0.5)^(i%2);
}
 
int CFrelement::save(char* name,bool lossy)
{
	FILE* file = fopen(name,"w");
	fwrite(&signalLength,sizeof(unsigned int),1,file);
	save(file,lossy);
	fclose(file);
}

int CFrelement::load(char* name)
{
	FILE* file = fopen(name,"r");
	if (fread(&signalLength,sizeof(unsigned int),1,file)!=1) return -1;
	load(file);
	fclose(file);
	return 0;
}


int CFrelement::save(FILE* file,bool lossy)
{
	unsigned int outlierNum = outliers;
	unsigned char code = 255;
	if (order == 0 && outliers == 0)
	{
		if (gain == 1) code=254;
		fwrite(&code,sizeof(unsigned char),1,file);
	}else{ 
		if (lossy) outlierNum = 0; 
		fwrite(&order,sizeof(unsigned char),1,file);
		fwrite(&outlierNum,sizeof(unsigned int),1,file);
		fwrite(&gain,sizeof(float),1,file);
		fwrite(frelements,sizeof(SFrelement),order,file);
		fwrite(outlierSet,sizeof(unsigned int),outlierNum,file);
	}
}

int CFrelement::load(FILE* file)
{
	int ret =0;
	ret+=fread(&order,sizeof(unsigned char),1,file);
	if (order>250){
		gain = 0;
		if (order == 254) gain = 1;
		outliers = 0;
		order = 0;
		if (ret != 1) ret = -1; else ret = 0;
	}else{
		ret+=fread(&outliers,sizeof(unsigned int),1,file);
		ret+=fread(&gain,sizeof(float),1,file);
		free(outlierSet);
		free(frelements);
		frelements = (SFrelement*) malloc(order*sizeof(SFrelement));
		//outlierSet = (unsigned int*)malloc(outliers*(sizeof(unsigned int)));
		ret+=fread(frelements,sizeof(SFrelement),order,file);
		ret+=fread(outlierSet,sizeof(unsigned int),outliers,file);
		if (ret != 4+outliers+order) ret = -1; else ret = 0;
	}
	return ret;
}


