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
	return active->estimate(time,map->data.data(),order);
}

int CFrelement2DGridSet::estimateEntropy(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map,int order)
{
	if (find(name) == false)return -1;
	return active->estimateEntropy(time,map->data.data(),order);
}

int CFrelement2DGridSet::evaluate(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map,int order,float errors[])
{
	if (find(name) == false) return -1;
	return active->evaluate(time,map->data.data(),order,errors);;
}

int CFrelement2DGridSet::print()
{
	for (int i =0;i<numGrids;i++) printf("Map %s is %ix%i\n",grids[i]->id,grids[i]->width,grids[i]->height);
}

int CFrelement2DGridSet::remove(const char *name)
{
	if (find(name) == false) return -numGrids;
	delete grids[activeIndex];
	grids[activeIndex] = grids[--numGrids];
	return numGrids+1;
}

int CFrelement2DGridSet::save(const char *name,const char *filename)
{
	if (find(name) == false) return -1;
	active->save(filename);
	return numGrids;
}

int CFrelement2DGridSet::load(const char *name,const char *filename)
{
	bool exists = find(name);
	if (exists == false){
		grids[numGrids++] = new CFrelement2DGrid(name);
		activeIndex = numGrids-1;
		active = grids[numGrids-1];
	}
	active->load(filename);
	return numGrids;
}
