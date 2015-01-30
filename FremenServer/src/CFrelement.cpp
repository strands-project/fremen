#include "CFrelement.h"

using namespace std;
static bool debug = false; 

int fremenSort(const void* i,const void* j) 
{
	 if (((SFrelement*)i)->amplitude < ((SFrelement*)j)->amplitude) return +1;
	 return -1;
}

CFrelement::CFrelement(const char* name)
{
	strcpy(id,name);

	//initialization of the frequency set
	for (int i=0;i<NUM_PERIODICITIES;i++) frelements[i].amplitude = frelements[i].phase = 0; 
	for (int i=0;i<NUM_PERIODICITIES;i++) frelements[i].period = (7*24*3600)/(i+1); 
	gain = 0;
	order = 0;
	firstTime = -1;
	lastTime = -1;
	measurements = 0;
}

CFrelement::~CFrelement()
{
}

// adds new state observations at given times
int CFrelement::add(uint32_t times[],unsigned char states[],int length)
{
	if (measurements == 0 && length > 0)
	{
		for (int i = 0;i<NUM_PERIODICITIES;i++){
			frelements[i].realStates = 0;
			frelements[i].imagStates = 0;
		}
		firstTime = times[0];
	}
	int duration = times[length-1]-firstTime;
	int firstIndex = 0;

	//discard already known observations 
	for (int i=0;i<length;i++)if (times[i] <= lastTime)firstIndex++;
	int numUpdated = length-firstIndex;
	lastTime = times[length-1];

	//verify if there is an actual update
	if (numUpdated <= 0)return numUpdated;

	//update the gains accordingly 
	float oldGain=0;
	float newGain=0;
	for (int j = firstIndex;j<length;j++)newGain+=states[j];
	gain = (gain*measurements+newGain)/(measurements+length);

	//recalculate spectral balance - this is beneficial is the process period does not match the length of the data
	if (oldGain > 0){
		for (int i = 0;i<NUM_PERIODICITIES;i++)
		{
			frelements[i].realBalance  = gain*frelements[i].realBalance/oldGain;
			frelements[i].imagBalance  = gain*frelements[i].imagBalance/oldGain;
		}
	}else{
		for (int i = 0;i<NUM_PERIODICITIES;i++)
		{
			frelements[i].realBalance  = 0;
			frelements[i].imagBalance  = 0;
		}
	}


	float angle = 0;
	//recalculate the spectral components
	for (int j = firstIndex;j<length;j++)
	{
		for (int i = 0;i<NUM_PERIODICITIES;i++)
		{
			angle = 2*M_PI*(float)times[j]/frelements[i].period;
			frelements[i].realStates   += states[j]*cos(angle);
			frelements[i].imagStates   += states[j]*sin(angle);
			frelements[i].realBalance  += gain*cos(angle);
			frelements[i].imagBalance  += gain*sin(angle);
		}
	}
	measurements+=length;

	//establish amplitudes and phase shifts
	float re,im;
	for (int i = 0;i<NUM_PERIODICITIES;i++)
	{
		re = frelements[i].realStates-frelements[i].realBalance;
		im = frelements[i].imagStates-frelements[i].imagBalance;
		if (1.5*frelements[i].period <= duration) frelements[i].amplitude = sqrt(re*re+im*im)/measurements; else frelements[i].amplitude = 0;
		if (frelements[i].amplitude < FREMEN_AMPLITUDE_THRESHOLD) frelements[i].amplitude = 0;
		//frelements[i].amplitude = sqrt(re*re+im*im)/measurements;
		frelements[i].phase = atan2(im,re);
	}

	//sort the spectral components
	qsort(frelements,NUM_PERIODICITIES,sizeof(SFrelement),fremenSort);

	return numUpdated; 
}

int CFrelement::evaluate(uint32_t* times,unsigned char* signal,int length,int orderi,float* evals)
{
	float estimate = 0;
	float time;
	unsigned char state;
	for (int j = 0;j<=orderi;j++) evals[j] = 0;
	for (int j = 0;j<length;j++)
	{
		time = times[j];
		state = signal[j];
		estimate = gain;
		evals[0]+= abs(state-(estimate>0.5));
		for (int i = 0;i<orderi;i++){
			 estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
			 evals[i+1]+= abs(state-(estimate>0.5));
		}
	}
	for (int j = 0;j<=order+1;j++) evals[j]=evals[j]/length;

	//get best model order
	float error = 10.0;
	int index = 0;
	for (int j = 0;j<=orderi;j++)
	{
		if (evals[j] < error-0.001){
			index = j;
			error = evals[j]; 
		}
	}
	return index;
}


/*not required in incremental version*/
void CFrelement::update(int modelOrder)
{
}

/*text representation of the fremen model*/
void CFrelement::print(bool verbose)
{
	int errs = 0;
	std::cout << "Model: " << id << " Prior: " << gain << " Size: " << measurements << " ";
	if (order > 0) std::cout  << endl;
	if (verbose){
		float ampl = gain;
		for (int i = 0;i<order;i++){
			std::cout << "Frelement " << i << " " << frelements[i].amplitude << " " << frelements[i].phase << " " << frelements[i].period << endl;
		}
	}
	std::cout << endl; 
}

int CFrelement::estimate(uint32_t times[],float probs[],int length,int orderi)
{
	float estimate = 0;
	float time;
	for (int j = 0;j<length;j++)
	{
		time = times[j];
		estimate = gain;
		for (int i = 0;i<orderi;i++) estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
		if (estimate > 1.0) estimate =  1.0;
		if (estimate < 0.0) estimate =  0.0;
		probs[j]=estimate;
	}
	return length;
}

int CFrelement::estimateEntropy(uint32_t times[],float entropy[],int length,int orderi)
{
	float estimate = 0;
	float time;
	for (int j = 0;j<length;j++)
	{
		time = times[j];
		estimate = gain;
		for (int i = 0;i<orderi;i++) estimate+=2*frelements[i].amplitude*cos(time/frelements[i].period*2*M_PI-frelements[i].phase);
		if (estimate <= 0 || estimate >= 1) entropy[j] = 0; else  entropy[j] = -(estimate*log2f(estimate)+(1-estimate)*log2f((1-estimate)));
	}
	return length;
}

int CFrelement::save(char* name,bool lossy)
{
	FILE* file = fopen(name,"w");
	save(file);
	fclose(file);
}

int CFrelement::load(char* name)
{
	FILE* file = fopen(name,"r");
	load(file);
	fclose(file);
	return 0;
}


int CFrelement::save(FILE* file,bool lossy)
{
	int frk = NUM_PERIODICITIES;
	fwrite(&frk,sizeof(uint32_t),1,file);
	fwrite(&gain,sizeof(float),1,file);
	fwrite(frelements,sizeof(SFrelement),NUM_PERIODICITIES,file);
	return 0;
}

int CFrelement::load(FILE* file)
{
	int frk = NUM_PERIODICITIES;
	fwrite(&frk,sizeof(uint32_t),1,file);
	fwrite(&gain,sizeof(float),1,file);
	fwrite(frelements,sizeof(SFrelement),NUM_PERIODICITIES,file);
	return 0;
}


