#include "CFrelementSet.h"

using namespace std;

CFrelementSet::CFrelementSet()
{
	numFrelements = 0;
	activeIndex = 0;
	active = NULL;
}

CFrelementSet::~CFrelementSet()
{
	for (int i=0;i<numFrelements;i++) delete frelements[i];
}

int CFrelementSet::add(const char *name,uint32_t times[],float states[],int length)
{
	bool exists = find(name);
	if (exists == false){
		frelements[numFrelements++] = new CFrelement(name);
		activeIndex = numFrelements-1;
		active = frelements[numFrelements-1];
	}
	//printf("Add %i %s \n",activeIndex,active->id);
	return active->add(times,states,length);
}

int CFrelementSet::evaluate(const char *name,uint32_t times[],unsigned char states[],int length,int order,float errors[])
{
	if (find(name) == false) return -1;
	return active->evaluate(times,states,length,order,errors);;
}

int CFrelementSet::detect(const char *name,uint32_t times[],float probs[],int length,int order,float confidence,uint32_t anomTimes[],float anomVals[])
{
	if (find(name) == false)return -1;
	return active->detect(times,probs,length,order,confidence,anomTimes,anomVals);
}

int CFrelementSet::estimate(const char *name,uint32_t times[],float probs[],int length,int order)
{
	if (find(name) == false)return -1;
	//printf("Estimate %i %s \n",activeIndex,active->id);
	return active->estimate(times,probs,length,order);
}

int CFrelementSet::estimateEntropy(const char *name,uint32_t times[],float entropy[],int length,int order)
{
	if (find(name) == false)return -1;
	//printf("Estimate %i %s \n",activeIndex,active->id);
	return active->estimateEntropy(times,entropy,length,order);
}

int CFrelementSet::getModelParameters(const char *name,float  periods[],float amplitudes[],float phases[],int order)
{
	if (find(name) == false)return -1;
	if (order > NUM_PERIODICITIES) order = NUM_PERIODICITIES;
	phases[0] = 0;
	periods[0] = +1.0/0.0;		//this corresponds to a maximum periodicity
	amplitudes[0] = active->gain;	//because the first element is the 'static' probability
	for (int i = 0;i<order;i++)
	{
		periods[i+1]=active->frelements[i].period;
		amplitudes[i+1]=active->frelements[i].amplitude;
		phases[i+1]=active->frelements[i].phase;
	}
	return order;
}

bool CFrelementSet::find(const char *name)
{
	int i = 0;
	for (i =0;(i<numFrelements) && (strcmp(frelements[i]->id,name)!=0);i++) {}
	if (i==numFrelements) return false;
	activeIndex = i;	
	active = frelements[i];
	return true;
}

int CFrelementSet::remove(const char *name)
{
	if (find(name) == false) return -numFrelements;
	delete frelements[activeIndex];
	frelements[activeIndex] = frelements[--numFrelements];
	return numFrelements+1;
}

bool CFrelementSet::update(const char* name,int order)
{
	if (find(name) == false) return false;
	active->update(order);
	return true;
}

bool CFrelementSet::print(int order)
{
	for (int i = 0;i<numFrelements;i++) frelements[i]->print(order);
}

bool CFrelementSet::load(FILE* file)
{
}

bool CFrelementSet::save(FILE* file)
{
}
