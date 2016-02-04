#include "CFrelement2DGrid.h"

using namespace std;
static bool debug = true;

extern float *periods;

CFrelement2DGrid::CFrelement2DGrid(const char* name)
{
	strcpy(id,name);
	numFrelements = 0;
	height = 0;
	width = 0;
	originX = 0;
	originY = 0;
	resolution = 0;
	frelementArray = NULL;
}

CFrelement2DGrid::~CFrelement2DGrid()
{
	if (frelementArray !=NULL){
		for (int i=0;i<numFrelements;i++){
			if (frelementArray[i] != NULL) delete frelementArray[i];
		}
		free(frelementArray);
	}
}

int CFrelement2DGrid::add(uint32_t time,int8_t states[],int widthi,int heighti,float originXi,float originYi,float resolutioni)
{
	int result = -1;
	/*is this a new map ? if yes, initialise all stuff*/
	if (numFrelements == 0)
	{
		height = heighti;
		width = widthi;
		numFrelements = height*width;
		originX = originXi;
		originY = originYi;
		resolution = resolutioni;
		frelementArray = (CFrelement**)malloc(numFrelements*sizeof(CFrelement*));
		for (int i = 0;i<numFrelements;i++) frelementArray[i] = NULL;
		result = 1;
	}
	if (height == heighti && width == widthi && resolution == resolutioni){ 
		for (int i = 0;i<numFrelements;i++)
		{
			if (states[i] != -1){
				if (frelementArray[i] == NULL) frelementArray[i] = new CFrelement();
				float signal = ((float)states[i])/100.0;
				frelementArray[i]->add(&time,&signal,1);
			}
		}
		result = 0;
	}
	return result; 
}

int CFrelement2DGrid::estimate(uint32_t time,int8_t states[],int order)
{
	float prob;
	for (int i = 0;i<numFrelements;i++)
	{
		if (frelementArray[i] == NULL){
			states[i] = -1;
		}else{
			frelementArray[i]->estimate(&time,&prob,1,order);
			states[i] = (int8_t)(100.0*prob);
		}
	}
	return 0;
}

int CFrelement2DGrid::estimateEntropy(uint32_t time,int8_t states[],int order)
{
	float prob;
	for (int i = 0;i<numFrelements;i++)
	{
		if (frelementArray[i] == NULL){
			states[i] = -1;
		}else{
			frelementArray[i]->estimateEntropy(&time,&prob,1,order);
			states[i] = (int8_t)(100.0*prob);
		}
	}
	return 0;
}

//not functional!
int CFrelement2DGrid::evaluate(uint32_t time,int8_t states[],int order,float errs[])
{
	for (int i = 0;i<=order;i++) errs[i] = 0;
	float evals[order+1];
	int numEvaluated = 0;
	for (int i = 0;i<numFrelements;i++)
	{
		if (states[i] != -1 && frelementArray[i] != NULL)
		{
			float signal = ((float)states[i])/100.0;
			frelementArray[i]->evaluate(&time,&signal,1,order,evals);
			for (int i = 0;i<=order;i++) errs[i] += evals[i];
			numEvaluated++;
		}
	}
	if (numEvaluated > 0) for (int i = 0;i<=order;i++) errs[i]=errs[i]/numEvaluated;
	
	int index = 0;
	float minError = 100000;
	for (int i = 0;i<=order;i++){
	       	if (errs[i]<minError)
		{
			index = i;	
			minError = errs[i];	
		}
	}
	return 0;
}

bool CFrelement2DGrid::print(int order)
{
	for (int i = 0;i<numFrelements;i++)
	{
		printf("Cell %i: ",i); 
		if (frelementArray[i] != NULL) frelementArray[i]->print(order); else printf("None\n"); 
	}
	return true;
}

int CFrelement2DGrid::load(const char* name)
{
	/*destroy old stuff*/
	if (frelementArray !=NULL){
		for (int i=0;i<numFrelements;i++){
			if (frelementArray[i] != NULL) delete frelementArray[i];
		}
		free(frelementArray);
	}

	/*read meta-information*/
	int result = 0;
	FILE* file=fopen(name,"r");
	if (file == NULL) return -1;
	result += fread(&numFrelements,sizeof(int),1,file);
	result += fread(&height,sizeof(int),1,file);
	result += fread(&width,sizeof(int),1,file);
	result += fread(&originX,sizeof(float),1,file);
	result += fread(&originY,sizeof(float),1,file);
	result += fread(&resolution,sizeof(float),1,file);
	if (debug) printf("%i %i %.3f %.3f %.3f %i\n",width,height,originX,originY,resolution,result);
	/*initialize array*/	
	frelementArray = (CFrelement**)malloc(numFrelements*sizeof(CFrelement*));
	for (int i = 0;i<numFrelements;i++) frelementArray[i] = NULL;
	unsigned char *occupancyArray = (unsigned char*)calloc(numFrelements/8+1,sizeof(unsigned char));
	result += fread(occupancyArray,sizeof(unsigned char),numFrelements/8+1,file);

	unsigned char a = 128;
	for (int i = 0;i<numFrelements;i++)
	{
		if ((occupancyArray[i/8]&a) > 0){
			frelementArray[i] = new CFrelement();
		       	frelementArray[i]->load(file);
		}
		if (a == 1) a=128; else a=a/2;
	}
	free(occupancyArray);
	fclose(file);
	return result;
}

int CFrelement2DGrid::save(const char* name)
{
	int result = 0;
	FILE* file=fopen(name,"w");
	if (file == NULL) return -1;

	/*the occupancy array allows to represent never-observed states as single bits*/
	int dummy = 0;
	unsigned char *occupancyArray = (unsigned char*)calloc(numFrelements/8+1,sizeof(unsigned char));
	unsigned char a = 128;
	for (int i = 0;i<numFrelements;i++)
	{
		if (frelementArray[i] != NULL) occupancyArray[i/8] = occupancyArray[i/8]|a; 
		if (a == 1) a=128; else a=a/2;
	}
	/*printf("Occupancy array:");
	for (int i = 0;i<=numFrelements/8;i++) printf("%x",occupancyArray[i]);
	printf("\n");*/
	unsigned char len = strlen(id);	
	result += fwrite(&len,sizeof(unsigned char),1,file);
	result += fwrite(id,len,1,file);
	result += fwrite(&numFrelements,sizeof(int),1,file);
	result += fwrite(&height,sizeof(int),1,file);
	result += fwrite(&width,sizeof(int),1,file);
	result += fwrite(&originX,sizeof(float),1,file);
	result += fwrite(&originY,sizeof(float),1,file);
	result += fwrite(&resolution,sizeof(float),1,file);
	result += fwrite(occupancyArray,sizeof(unsigned char),numFrelements/8+1,file);
	if (debug) printf("Map %s with dimensions %ix%i, origin %.3fx%.3f and resolution %.3f saved in file %s.\n",id,width,height,originX,originY,resolution,name);
	for (int i = 0;i<numFrelements;i++)
	{
		if (frelementArray[i] != NULL) result += frelementArray[i]->save(file);  
	}
	free(occupancyArray);
	fclose(file);
	return result;
}
