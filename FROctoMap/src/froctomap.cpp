#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOccupancyOcTree.h>	
#include "CFremenGrid.h"
#include "CTimer.h"

#include "fremen/UpdateGrid.h"
#include "fremen/SaveGrid.h"
#include "fremen/EvaluateGrid.h"
#include "fremen/RecoverOctomap.h"
#include "fremen/EstimateOctomap.h"
#include <std_msgs/String.h>

/*
#define DIM_X 201 
#define DIM_Y 201
#define DIM_Z 101
#define LIM_MIN_X -5.0
#define LIM_MIN_Y -5.0
#define LIM_MIN_Z -3.0
#define LIM_MAX_X 5.0
#define LIM_MAX_Y 5.0
#define LIM_MAX_Z 2.0*/

/*delimit the OctoMap dimensions here */
#define DIM_X 54 
#define DIM_Y 42
#define DIM_Z 94 
#define LIM_MIN_X -2.6
#define LIM_MIN_Y -2.0
#define LIM_MIN_Z 0.0
#define LIM_MAX_X 0.05
#define LIM_MAX_Y 0.05
#define LIM_MAX_Z 4.7

using namespace std;
using namespace octomap;

bool debug = false;
int indices[1000000];
char gridka[DIM_X*DIM_Y*DIM_Z];
char gridkc[DIM_X*DIM_Y*DIM_Z];
float gridkp[DIM_X*DIM_Y*DIM_Z];
float lastPoseX,lastPoseY;
float mapPoseX,mapPoseY;
char lastNode[100];
char mapNode[100];
int currentMap = -1;
int numMaps = 0;
int poseCount = 0;
CFremenGrid *gridArray[10];
CFremenGrid *gridPtr;
ros::Publisher *octomap_pub_ptr, *retrieve_pub_ptr,*estimate_pub_ptr,*clock_pub_ptr;

//Parameters
double resolution, m_colorFactor;
string filename;
string loadfilename;

std_msgs::ColorRGBA heightMapColor(double h)
{
	std_msgs::ColorRGBA color;
	color.a = 1.0;
	color.g = h;	
	color.r = 1-h;	
	color.b = 0;	
	return color;
}

std_msgs::ColorRGBA heightMapColorA(double h){

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
    
  }
  
  return color;
}


void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg)
{
	currentMap = -1;
	for (int i =0;i<numMaps;i++){
		if (strcmp(mapNode,gridArray[i]->name)==0) currentMap = i; 
	}
	if (currentMap == -1){
		currentMap = numMaps;
		gridArray[numMaps++] = new CFremenGrid(DIM_X,DIM_Y,DIM_Z);
		gridArray[currentMap]->setName(mapNode);
		gridArray[currentMap]->setPose(mapPoseX,mapPoseY);
		ROS_INFO("Map %s does not exist, creating map at %f %f.",mapNode,mapPoseX,mapPoseY);
		gridPtr = gridArray[currentMap];
	}else{
		gridPtr = gridArray[currentMap];
	}

	AbstractOcTree* tree = msgToMap(*msg);
	if(tree){
		OcTree* octree = dynamic_cast<OcTree*>(tree);
		int cnt = 0;

		double minX, minY, minZ, maxX, maxY, maxZ;
		octree->getMetricMin(minX, minY, minZ);
		octree->getMetricMax(maxX, maxY, maxZ);
		if (debug) printf("%f %f %f\n",minX,minY,minZ);
		if (debug) printf("%f %f %f\n",maxX,maxY,maxZ);
		float x = 0*gridPtr->positionX; 
		float y = 0*gridPtr->positionY;
		if (debug) printf("%f %f %f\n",LIM_MIN_X,LIM_MIN_Y,LIM_MIN_Z);
		if (debug) printf("%f %f %f\n",LIM_MAX_X,LIM_MAX_Y,LIM_MAX_Z);
		for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution){
			for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution){
				for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution){
					OcTreeNode* n = octree->search(i+x,j+y,w);
					if(n){
						if(octree->isNodeOccupied(n))
						{
							gridPtr->add(cnt,1);
						}
						else
						{
							gridPtr->add(cnt,0);
						}
					}else{
						gridPtr->add(cnt,0);
					}
					cnt++;
				}
			}
		}
		ROS_INFO("3D Grid Updated! -> Iteration: %i over %i elements in %i element grid", gridPtr->signalLength,cnt,gridPtr->numCells);
	}else{
		ROS_ERROR("Octomap conversion error!");
		exit(1);
	}
	gridPtr->signalLength++;
	delete tree;
}

bool load_octomap(fremen::SaveGrid::Request  &req, fremen::SaveGrid::Response &res)
{
	currentMap = -1;
	for (int i =0;i<numMaps;i++)
	{
		if (strcmp(req.mapname.c_str(),gridArray[i]->name)==0) currentMap = i; 
	}
	if (currentMap == -1)
	{
		ROS_INFO("Map %s does not exist!",req.mapname.c_str());
		currentMap = numMaps;
		gridArray[numMaps++] = new CFremenGrid(DIM_X,DIM_Y,DIM_Z);
		gridPtr = gridArray[currentMap];
	}else{
		gridPtr = gridArray[currentMap];
	}
	gridPtr->load(req.filename.c_str());
	ROS_INFO("3D Grid of %i loaded !",gridPtr->signalLength);
	res.result = true;
//	gridPtr->print(true);
	return true;
}

bool evaluate_octomap(fremen::EvaluateGrid::Request  &req, fremen::EvaluateGrid::Response &res)
{
	currentMap = -1;
	for (int i =0;i<numMaps;i++){
		if (strcmp(req.mapname.c_str(),gridArray[i]->name)==0) currentMap = i; 
	}
	if (currentMap == -1){
		ROS_INFO("Map %s does not exist!",req.mapname.c_str());
		res.result = false;
		gridPtr = NULL;
		return false;
	}else{
		gridPtr = gridArray[currentMap];
	}
	SGridErrors e[req.maxOrder+1];
	gridPtr->evaluatePrecision(e,req.maxOrder+1);
	res.size = gridPtr->signalLength;
	res.allErrors.clear();  
	for (int i = 0;i<req.maxOrder+1;i++)
	{
		ROS_INFO("Error rate order (all/nonempty/dynamic) is %.5f/%.5f/%.5f.",i,e[i].all,e[i].nonempty,e[i].dynamic);
		res.allErrors.push_back(e[i].all);  
		res.nonemptyErrors.push_back(e[i].nonempty);  
		res.dynamicErrors.push_back(e[i].dynamic);  
	} 
	ROS_INFO("3D Grid of length %i saved!",res.size);
	res.result = true;
	return true;
}

bool save_octomap(fremen::SaveGrid::Request  &req, fremen::SaveGrid::Response &res)
{
	currentMap = -1;
	for (int i =0;i<numMaps;i++){
		if (strcmp(req.mapname.c_str(),gridArray[i]->name)==0) currentMap = i; 
	}
	if (currentMap == -1){
		ROS_INFO("Map %s does not exist!",req.mapname.c_str());
		res.result = false;
		gridPtr = NULL;
		return false;
	}else{
		gridPtr = gridArray[currentMap];
	}
	
	gridPtr->save(req.filename.c_str(), (bool) req.lossy,req.order);
	res.size = gridPtr->signalLength;
	ROS_INFO("3D Grid of length %ld saved!",res.size);
    res.result = true;
	return true;
}

bool update_octomap(fremen::UpdateGrid::Request  &req, fremen::UpdateGrid::Response &res)
{
	currentMap = -1;
	for (int i =0;i<numMaps;i++){
		if (strcmp(req.mapname.c_str(),gridArray[i]->name)==0) currentMap = i; 
	}
	if (currentMap == -1){
		ROS_INFO("Map %s does not exist!",req.mapname.c_str());
		res.result = false;
		gridPtr = NULL;
		return false;
	}else{
		gridPtr = gridArray[currentMap];
	}
	SGridErrors errors = gridPtr->update((int) req.order, gridPtr->signalLength,true);
	res.size = gridPtr->signalLength;
	ROS_INFO("3D Grid updated - error rate (all/nonempty/dynamic) is %.5f/%.5f/%.5f.",errors.all,errors.nonempty,errors.dynamic);
	res.allError = errors.all;
	res.dynamicError = errors.dynamic;
	res.nonemptyError = errors.nonempty;
	res.result = true;
	return true;
}

bool recover_octomap(fremen::RecoverOctomap::Request  &req, fremen::RecoverOctomap::Response &res)
{
	currentMap = -1;
	for (int i =0;i<numMaps;i++){
		if (strcmp(req.mapname.c_str(),gridArray[i]->name)==0) currentMap = i; 
	}
	if (currentMap == -1){
		ROS_INFO("Map %s does not exist!",req.mapname.c_str());
		res.result = false;
		gridPtr = NULL;
		return false;
	}else{
		gridPtr = gridArray[currentMap];
	}
	
  octomap_msgs::Octomap bmap_msg;
  OcTree octree (resolution);
  ROS_INFO("Service: recover stamp %.3f",req.stamp);

  //Create pointcloud:
  octomap::Pointcloud octoCloud;
  sensor_msgs::PointCloud fremenCloud;
  float x = 0*gridPtr->positionX; 
  float y = 0*gridPtr->positionY; 
  geometry_msgs::Point32 test_point;
  int cnt = 0;
  for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution){
	  for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution){
		  for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution){
			  point3d ptt(x+i+resolution/2,y+j+resolution/2,w+resolution/2);
			  
			  if(gridPtr->retrieve(cnt, req.stamp))
			  {
				  octoCloud.push_back(x+i+resolution/2,y+j+resolution/2,w+resolution/2);
				  octree.updateNode(ptt,true,true);
			  }
			  cnt++;
		  }
	  }
  }
  //Update grid
  octree.updateInnerOccupancy();
  
  //init visualization markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  unsigned int m_treeDepth = octree.getTreeDepth();
  
  //each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth + 1);
  geometry_msgs::Point cubeCenter;
  
  std_msgs::ColorRGBA m_color;
  m_color.r = 0.0;
  m_color.g = 0.0;
  m_color.b = 1.0;
  m_color.a = 1.0;
  
  for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) 
  {
    double size = octree.getNodeSize(i);
    occupiedNodesVis.markers[i].header.frame_id = "/map";
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].color = m_color;
  }
  x = gridPtr->positionX; 
  y = gridPtr->positionY; 
  for(OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
  {
	  if(it != NULL && octree.isNodeOccupied(*it))
	  {
		  unsigned idx = it.getDepth();
		  cubeCenter.x = x+it.getX();
		  cubeCenter.y = y+it.getY();
		  cubeCenter.z = 1.45+it.getZ();
		  occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
		  double minX, minY, minZ, maxX, maxY, maxZ;
		  octree.getMetricMin(minX, minY, minZ);
		  octree.getMetricMax(maxX, maxY, maxZ);
		  double h = (1.0 - fmin(fmax((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
		  occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
	  }
  } 
  octomap_msgs::binaryMapToMsg(octree, bmap_msg);
  
  fremenCloud.header.frame_id = "/map";
  fremenCloud.header.stamp = ros::Time::now();
  
  octomap_pub_ptr->publish(bmap_msg);
  retrieve_pub_ptr->publish(occupiedNodesVis);
  
  ROS_INFO("Octomap published!");
  
  res.result = true;

  return true;
}

bool expandNode()
{

}


bool calculateClusters(int seed,int threshold)
{
	int endIndices,startIndices;
	int dimX = (int)((LIM_MAX_X-LIM_MIN_X)/resolution);
	int dimY = (int)((LIM_MAX_Y-LIM_MIN_Y)/resolution);
	int dimZ = (int)((LIM_MAX_Z-LIM_MIN_Z)/resolution);
	int nn[] = {1,-1,dimZ,-dimZ,dimY*dimZ,-dimY*dimZ};
	int siz = dimX*dimY*dimZ;
	int cur,exp;
	memset(gridkc,0,sizeof(char)*siz);
	for (int i = 0;i<siz;i++)
	{
		endIndices=startIndices=0;
		if (gridka[i] > seed && gridkc[i] == 0){
			 printf("GRIDINDEX: %i\n",i);
			 indices[endIndices++] = i;
			 while (endIndices-startIndices > 0){
				 cur=indices[startIndices++];
				 for (int j=0;j<6;j++)
				 {
					exp = cur+nn[j];
					if (gridkc[exp]==0 && gridka[exp]>threshold){
						 indices[endIndices++]=exp;
						 gridkc[exp]=1;
					}
				 }
			 }
			 printf("GRIDSIZE: %i %i\n",i,endIndices);
		}
		if (endIndices > 20){
			for (int j=0;j<endIndices;j++)	gridka[indices[j]] = seed+1;
		}
	}
}

bool estimate_octomap(fremen::EstimateOctomap::Request  &req, fremen::EstimateOctomap::Response &res)
{
	currentMap = -1;
	for (int i =0;i<numMaps;i++){
		if (strcmp(req.mapname.c_str(),gridArray[i]->name)==0) currentMap = i; 
	}
	if (currentMap == -1){
		ROS_INFO("Map %s does not exist!",req.mapname.c_str());
		res.result = false;
		gridPtr = NULL;
		return false;
	}else{
		gridPtr = gridArray[currentMap];
	}

    ROS_INFO("Service: estimate stamp %.3f",req.stamp);
  octomap_msgs::Octomap bmap_msg;
  OcTree octree (resolution);
  
  //Create pointcloud:
  octomap::Pointcloud octoCloud;
  sensor_msgs::PointCloud fremenCloud;
  
  geometry_msgs::Point32 test_point;
  int cnt = 0;
  float estimate;
  float x = 0*gridPtr->positionX; 
  float y = 0*gridPtr->positionY; 
  for (int i = 0;i<DIM_X*DIM_Y*DIM_Z;i++){
	  estimate = gridPtr->fineEstimate(i, req.stamp);
	  gridkp[i] = estimate;
	  if(estimate >req.minProbability && estimate< req.maxProbability) gridka[i] = 1; else gridka[i] = 0;
  }
  int dimX = (int)((LIM_MAX_X-LIM_MIN_X)/resolution);
  int dimY = (int)((LIM_MAX_Y-LIM_MIN_Y)/resolution);
  int dimZ = (int)((LIM_MAX_Z-LIM_MIN_Z)/resolution);
  int nn[] = {1,-1,dimZ,-dimZ,dimY*dimZ,-dimY*dimZ};
  for (int i = dimY*dimZ;i<dimX*dimY*dimZ-dimY*dimZ;i++){
	  if (gridka[i] == 1){ 
		  for (int j = 0;j<6;j++){
			  if (gridka[i+nn[j]]>0) gridka[i]++;
		  }
	  }
  }
  calculateClusters(req.morphology,3);
  for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution){
	  for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution){
		  for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution){
			  if(gridka[cnt] > req.morphology)
			  {
				  point3d ptt(x+i+resolution/2,y+j+resolution/2,w+resolution/2);
				  octoCloud.push_back(x+i+resolution/2,y+j+resolution/2,w+resolution/2);
				  octree.updateNode(ptt,true,true);
			  }
			  cnt++;
		  }
	  }
  }
  //Update grid
  octree.updateInnerOccupancy();
  
  visualization_msgs::Marker clockVis;
  //CLOCK
  clockVis.header.frame_id = "/map";
  clockVis.header.stamp = ros::Time(0);
  clockVis.ns = "my_namespace";
  clockVis.id = 0;
  clockVis.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  clockVis.action = visualization_msgs::Marker::ADD;
  clockVis.pose.position.x = gridPtr->positionX;
  clockVis.pose.position.y = gridPtr->positionY;
  clockVis.pose.position.z = 5.0;
  clockVis.pose.orientation.x = 0.0;
  clockVis.pose.orientation.y = 0.0;
  clockVis.pose.orientation.z = 0.0;
  clockVis.pose.orientation.w = 1.0;
  clockVis.scale.x = 0;
  clockVis.scale.y = 0;
  clockVis.scale.z = 0.5;
  clockVis.color.a = 1.0;
  clockVis.color.r = 1.0;
  clockVis.color.g = 1.0;
  clockVis.color.b = 1.0;
  char timeStr[1000];
  sprintf(timeStr,"Time:  %02i:%02i\n",12+(int)req.stamp,((int)(req.stamp*60))%60);
  if (req.maxProbability>1.0)sprintf(timeStr,"%sDisplay: all",timeStr); else if (req.morphology == 0) sprintf(timeStr,"%sDisplay: dynamic",timeStr); else sprintf(timeStr,"%sDisplay: periodic",timeStr);
  clockVis.text = timeStr;
  clock_pub_ptr->publish(clockVis);
  //init visualization markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  unsigned int m_treeDepth = octree.getTreeDepth();
  
  //each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth + 1);
  geometry_msgs::Point cubeCenter;
  
  std_msgs::ColorRGBA m_color;
  m_color.r = 0.0;
  m_color.g = 0.0;
  m_color.b = 1.0;
  m_color.a = 1.0;
  
  for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) 
  {
    double size = octree.getNodeSize(i);
    occupiedNodesVis.markers[i].header.frame_id = "/map";
    occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].scale.x = size;
    occupiedNodesVis.markers[i].scale.y = size;
    occupiedNodesVis.markers[i].scale.z = size;
    occupiedNodesVis.markers[i].color = m_color;
  }

  printf("B %i\n",cnt);
  x = gridPtr->positionX; 
  y = gridPtr->positionY;
  printf("Bl %i\n",cnt);
  cnt = 0;
/*  for(double i = LIM_MIN_X; i < LIM_MAX_X; i+=resolution){
	  for(double j = LIM_MIN_Y; j < LIM_MAX_Y; j+=resolution){
		  for(double w = LIM_MIN_Z; w < LIM_MAX_Z; w+=resolution){
			  printf("Bla %i\n",cnt);
			  if(gridkp[cnt] > req.morphology)
			  {
	printf("Blb\n");
				  cubeCenter.x =x+i+resolution/2; 
				  cubeCenter.y =y+j+resolution/2;
				  cubeCenter.z =1.45+w+resolution/2; 
				//  occupiedNodesVis.markers[16].points.push_back(cubeCenter);
				  double h = m_colorFactor;
				//  occupiedNodesVis.markers[16].colors.push_back(heightMapColor(h));
			  }
			  cnt++;
		  }
	  }
  }*/
 
  for(OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
  {
	  if(it != NULL && octree.isNodeOccupied(*it))
	  {
		  unsigned idx = it.getDepth();
		  cubeCenter.x = x+it.getX();
		  cubeCenter.y = y+it.getY();
		  cubeCenter.z = 1.45+it.getZ();
		  occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
		  cnt = (int)((it.getX()-LIM_MIN_X)/resolution)*dimY*dimZ+(int)((it.getY()-LIM_MIN_Y)/resolution)*dimZ+(int)((it.getZ()-LIM_MIN_Z)/resolution);
		  /*double minX, minY, minZ, maxX, maxY, maxZ;
		  octree.getMetricMin(minX, minY, minZ);
		  octree.getMetricMax(maxX, maxY, maxZ);
		  double h = (1.0 - fmin(fmax((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;*/
		 // if (gridkp[cnt]<1.0)  printf("CNT:%f %i\n",gridkp[cnt],cnt);
		  occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(gridkp[cnt]));
	  }
  }
  octomap_msgs::binaryMapToMsg(octree, bmap_msg);
  
  fremenCloud.header.frame_id = "/map";
  fremenCloud.header.stamp = ros::Time::now();
  
  octomap_pub_ptr->publish(bmap_msg);
  estimate_pub_ptr->publish(occupiedNodesVis);
  
  ROS_INFO("Octomap published!");
  
  res.result = true;
  return true;
}

void pose_cb (const geometry_msgs::PosePtr& pose)
{
	if ((int)(lastPoseX*100) == (int)(pose->position.x*100) && (int)(lastPoseY*100) == (int)(pose->position.y*100))
	{
		//ROS_INFO("Position stable %i %f %f\n",poseCount,pose->position.x,pose->position.y);
		poseCount++; 
	}else{
		//ROS_INFO("Position unstable %i %f %f\n",poseCount,pose->position.x,pose->position.y);
		poseCount=0; 
	}
	if (poseCount == 20){
		mapPoseX = lastPoseX;
		mapPoseY = lastPoseY;
		strcpy(mapNode,lastNode);
		ROS_INFO("Position updated %f %f\n",pose->position.x,pose->position.y);
	}
	lastPoseX = pose->position.x;
	lastPoseY = pose->position.y;
}

void name_cb (const std_msgs::StringPtr& input)
{
	strcpy(lastNode,input->data.c_str());
}

int main(int argc,char *argv[])
{
	CFFTPlan plan;
	CTimer timer;
	timer.start();
	plan.prepare(200000);
	printf("%i\n",timer.getTime());
	timer.reset();
	plan.prepare(200000);
	printf("%i\n",timer.getTime());
	return 0;

	ros::init(argc, argv, "froctomap");
	ros::NodeHandle n;

	ros::NodeHandle nh("~");
	nh.param("resolution", resolution, 0.1);
	nh.param("colorFactor", m_colorFactor, 0.8);
	nh.param<std::string>("filename", loadfilename, "");
	strcpy(mapNode,"Greg_office");

	//Fremen Grid:
	gridPtr = NULL;
/*	if (strlen(loadfilename.c_str()) > 0){
		numMaps = 0;
		printf("attempt to load\n");
		gridArray[numMaps++] = new CFremenGrid(DIM_X,DIM_Y,DIM_Z);
		currentMap = 0;
		if (gridArray[0]->load(loadfilename.c_str())){
			gridPtr = gridArray[currentMap];
		}else{
			numMaps = 0;
			gridPtr = NULL;
		}
	}*/
	//Subscribers:
	ros::Subscriber sub_octo = n.subscribe("/octomap_binary", 1000, octomap_cb);
//	ros::Subscriber sub_pose = n.subscribe ("/robot_pose", 1, pose_cb);
//	ros::Subscriber sub_name = n.subscribe ("/current_node", 1, name_cb);

	//Publishers:
	ros::Publisher octomap_pub = n.advertise<octomap_msgs::Octomap>("/froctomap", 100);
	octomap_pub_ptr = &octomap_pub;

	ros::Publisher retrieve_pub = n.advertise<visualization_msgs::MarkerArray>("/froctomap_recovered", 100);
	retrieve_pub_ptr = &retrieve_pub;

	ros::Publisher estimate_pub = n.advertise<visualization_msgs::MarkerArray>("/froctomap_estimate", 100);
	estimate_pub_ptr = &estimate_pub;
	
	ros::Publisher clock_pub = n.advertise<visualization_msgs::Marker>("/froctomap_clock", 100);
	clock_pub_ptr = &clock_pub;

	//Services:
	ros::ServiceServer retrieve_service = n.advertiseService("recover_octomap", recover_octomap);
	ros::ServiceServer estimate_service = n.advertiseService("estimate_octomap", estimate_octomap);
	ros::ServiceServer save_service = n.advertiseService("save_grid", save_octomap);
	ros::ServiceServer load_service = n.advertiseService("load_grid", load_octomap);
	ros::ServiceServer evaluate_service = n.advertiseService("evaluate_grid", evaluate_octomap);
	ros::ServiceServer update_service = n.advertiseService("update_grid", update_octomap);

	ros::spin();

	return 0;
}
