#include "CFrelement2DGridSet.h"

using namespace std;

CFrelement2DGridSet::CFrelement2DGridSet()
{
	numGrids = 0;
	activeIndex = 0;
	active = NULL;
}

CFrelement2DGridSet::~CFrelement2DGridSet()
{
	for (int i=0;i<numGrids;i++) delete grids[i];
}

int CFrelement2DGridSet::add(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map)
{
	bool exists = find(name);
	if (exists == false){
		grids[numGrids++] = new CFrelement2DGrid(name);
		activeIndex = numGrids-1;
		active = grids[numGrids-1];
	}
	return active->add(time,map->data.data(),map->info.width,map->info.height,map->info.origin.position.x,map->info.origin.position.y,map->info.resolution);
}

bool CFrelement2DGridSet::find(const char *name)
{
	int i = 0;
	for (i =0;(i<numGrids) && (strcmp(grids[i]->id,name)!=0);i++) {}
	if (i==numGrids) return false;
	activeIndex = i;	
	active = grids[i];
	return true;
}

int CFrelement2DGridSet::estimate(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map,int order)
{
	if (find(name) == false)return -1;
	//printf("Estimate %i %s \n",activeIndex,active->id);
	return active->estimate(time,map->data.data(),order);
}

/*
int CFrelement2DGridSet::evaluate(const char *name,uint32_t times[],unsigned char states[],int length,int order,float errors[])
{
	if (find(name) == false) return -1;
	return active->evaluate(times,states,length,order,errors);;
}



int CFrelement2DGridSet::estimateEntropy(const char *name,uint32_t times[],float entropy[],int length,int order)
{
	if (find(name) == false)return -1;
	return active->estimateEntropy(times,entropy,length,order);
}



int CFrelement2DGridSet::remove(const char *name)
{
	if (find(name) == false) return -numGrids;
	delete grids[activeIndex];
	grids[activeIndex] = grids[--numGrids];
	return numGrids+1;
}*/
