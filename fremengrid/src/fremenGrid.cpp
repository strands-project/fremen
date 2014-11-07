#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "CFremenGrid.h"
#include "CTimer.h"

#include <fremengrid/Entropy.h>
#include <fremengrid/SaveLoad.h>
#include <fremengrid/AddView.h>
#include <fremengrid/Visualize.h>
#include <std_msgs/String.h>

#define BHAM_SMALL 

#ifdef BHAM_LARGE
#define MIN_X  -18.2
#define MIN_Y  -31.0 
#define MIN_Z  0.0
#define DIM_X 560 
#define DIM_Y 990 
#define DIM_Z 60 
#endif

#ifdef BHAM_SMALL
#define MIN_X  -5.8
#define MIN_Y  -19.0 
#define MIN_Z  0.0
#define DIM_X 250 
#define DIM_Y 500 
#define DIM_Z 80 
#endif

#ifdef UOL_SMALL 
#define MIN_X  -7
#define MIN_Y  -5.6 
#define MIN_Z  0.0
#define DIM_X 280 
#define DIM_Y 450 
#define DIM_Z 80 
#endif

#define CAMERA_RANGE 4.0 

#define RESOLUTION 0.05 

using namespace std;

bool debug = false;
int integrateMeasurements = 0;

CFremenGrid *grid;
tf::TransformListener *tf_listener;

ros::Publisher *octomap_pub_ptr, *estimate_pub_ptr,*clock_pub_ptr;
ros::Publisher retrieve_publisher;
 
bool loadGrid(fremengrid::SaveLoad::Request  &req, fremengrid::SaveLoad::Response &res)
{
    grid->load(req.filename.c_str());
    ROS_INFO("3D Grid of %ix%ix%i loaded !",grid->xDim,grid->yDim,grid->zDim);
    res.result = true;
    return true;
}

bool saveGrid(fremengrid::SaveLoad::Request  &req, fremengrid::SaveLoad::Response &res)
{
    grid->save(req.filename.c_str(), (bool) req.lossy,req.order);
    ROS_INFO("3D Grid of %ix%ix%i saved !",grid->xDim,grid->yDim,grid->zDim);
    return true;
}

void points(const sensor_msgs::PointCloud2ConstPtr& points2)
{
	CTimer timer;
	if (integrateMeasurements == 20){
		timer.reset();
		timer.start();
		sensor_msgs::PointCloud points1,points;
		sensor_msgs::convertPointCloud2ToPointCloud(*points2,points1);
		tf::StampedTransform st;
		try {
			tf_listener->waitForTransform("/map","/head_xtion_depth_optical_frame",points2->header.stamp, ros::Duration(1.0));
			tf_listener->lookupTransform("/map","/head_xtion_depth_optical_frame",points2->header.stamp,st);
		}
		catch (tf::TransformException ex) {
			ROS_ERROR("FreMEn map cound not incorporate the latest measurements %s",ex.what());
		        return;
		}
		printf("Transform arrived %i \n",timer.getTime());	
		timer.reset();	
		tf_listener->transformPointCloud("/map", points1,points);
		int size=points.points.size();
		std::cout << "There are " << size << " fields." << std::endl;
		std::cout << "Point " << st.getOrigin().x() << " " <<  st.getOrigin().y() << " " << st.getOrigin().z() << " " << std::endl;
		float x[size+1],y[size+1],z[size+1],d[size+1];
		int last = 0;
		for(unsigned int i = 0; i < size; i++){
			if (isnormal(points.points[i].x) > 0)
			{	
				x[last] = points.points[i].x; 
				y[last] = points.points[i].y;
				z[last] = points.points[i].z;
				d[last] = 1;
				last++;
			}
		}
		x[last] = st.getOrigin().x();
		y[last] = st.getOrigin().y();
		z[last] = st.getOrigin().z();
		printf("Point cloud processed %i \n",timer.getTime());
		timer.reset();	
		grid->incorporate(x,y,z,d,last);
		printf("Grid updated %i \n",timer.getTime());	
		integrateMeasurements--;
	}
	if (integrateMeasurements == 1)
	{
		integrateMeasurements--;
		fremengrid::Visualize::Request req;
		req.green = req.blue = 0.0;
		req.red = req.alpha = 1.0;
	}
}

bool addView(fremengrid::AddView::Request  &req, fremengrid::AddView::Response &res)
{
	integrateMeasurements = 2;
	res.result = true;
	return true;
}

bool addDepth(fremengrid::AddView::Request  &req, fremengrid::AddView::Response &res)
{
	integrateMeasurements = 3;
	res.result = true;
	return true;
}

bool estimateEntropy(fremengrid::Entropy::Request  &req, fremengrid::Entropy::Response &res)
{
	res.value = grid->getInformation(req.x,req.y,req.z,M_PI,0.4,req.r,req.t);
	return true;
}

bool visualizeGrid(fremengrid::Visualize::Request  &req, fremengrid::Visualize::Response &res)
{
	//init visualization markers:
	visualization_msgs::Marker markers;
	geometry_msgs::Point cubeCenter;

	/// set color
	std_msgs::ColorRGBA m_color;
	m_color.r = req.red;
	m_color.g = req.green;
	m_color.b = req.blue;
	m_color.a = req.alpha;
	markers.color = m_color;

	/// set type, frame and size 
	markers.header.frame_id = "/map";
	markers.header.stamp = ros::Time::now();
	markers.ns = req.name;
	markers.action = visualization_msgs::Marker::ADD;
	markers.type = visualization_msgs::Marker::CUBE_LIST;
	markers.scale.x = RESOLUTION;
	markers.scale.y = RESOLUTION;
	markers.scale.z = RESOLUTION;
	markers.points.clear();

	// prepare to iterate over the entire grid 
	float minX = MIN_X;
	float minY = MIN_Y;
	float minZ = MIN_Z;
	float maxX = MIN_X+DIM_X*RESOLUTION-3*RESOLUTION/4;
	float maxY = MIN_Y+DIM_Y*RESOLUTION-3*RESOLUTION/4;
	float maxZ = MIN_Z+DIM_Z*RESOLUTION-3*RESOLUTION/4;
	int cnt = 0;
	int cells = 0;
	float estimate,minP,maxP;
	minP = req.minProbability;
	maxP = req.maxProbability;
	
	//iterate over the cells' probabilities 
	for(float z = MIN_Z;z<maxZ;z+=RESOLUTION){
		for(float y = MIN_Y;y<maxY;y+=RESOLUTION){
			for(float x = MIN_X;x<maxX;x+=RESOLUTION){
				if (req.type == 0) estimate = grid->estimate(cnt,0);
				if (req.type == 1) estimate = grid->aux[cnt];
				
				if(estimate > minP && estimate < maxP)
				{
					cubeCenter.x = x;
					cubeCenter.y = y;
					cubeCenter.z = z;
					markers.points.push_back(cubeCenter);
					markers.colors.push_back(m_color);
					cells++;
				}
				cnt++;
			}
		}
	}

	//publish results 
	retrieve_publisher.publish(markers);
	res.number = cells;
	return true;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	float depth = msg->data[640*480+640]+256*msg->data[640*480+640+1];
	int len =  msg->height*msg->width;
	float vx = 1/570.0; 
	float vy = 1/570.0; 
	float cx = -320.5;
	float cy = -240.5;
	int width = msg->width;
	int height = msg->height;
	float fx = (1+cx)*vx;
	float fy = (1+cy)*vy;
	float lx = (width+cx)*vx;
	float ly = (height+cy)*vy;
	float x[len+1];
	float y[len+1];
	float z[len+1];
	float d[len+1];
	float di,psi,phi,phiPtu,psiPtu,xPtu,yPtu,zPtu,ix,iy,iz;
	int cnt = 0;
	di=psi=phi=phiPtu=psiPtu=xPtu=yPtu=zPtu=0;

	if (integrateMeasurements == 3)
	{
		tf::StampedTransform st;
		try {
			tf_listener->waitForTransform("/map","/head_xtion_depth_optical_frame",msg->header.stamp, ros::Duration(0.5));
			tf_listener->lookupTransform("/map","/head_xtion_depth_optical_frame",msg->header.stamp,st);
		}
		catch (tf::TransformException ex) {
			ROS_ERROR("FreMEn map cound not incorporate the latest depth map %s",ex.what());
			return;
		}
		x[len] = xPtu = st.getOrigin().x();
		y[len] = yPtu = st.getOrigin().y();
		z[len] = zPtu = st.getOrigin().z();
		double a,b,c;	
		tf::Matrix3x3  rot = st.getBasis();
		rot.getEulerYPR(a,b,c,1);
		printf("%.3f %.3f %.3f\n",a,b,c);
		phi = a-c;
		psi = b;
		for (float h = fy;h<ly;h+=vy)
		{
			for (float w = fx;w<lx;w+=vx)
			{
				di = (msg->data[cnt*2]+256*msg->data[cnt*2+1])/1000.0;
				//printf("%f.3\n",di);
				d[cnt] = 1;
				if (di < 0.05){
					di = CAMERA_RANGE;
					d[cnt] = 0;
				}
				ix = di;
				iy = -w*di;
				iz = -h*di;
				x[cnt] = cos(phi)*ix-sin(phi)*iy+xPtu;
				y[cnt] = sin(phi)*ix+cos(phi)*iy+yPtu;
				z[cnt] = iz+zPtu;
				cnt++;	
			}
		}
		grid->incorporate(x,y,z,d,len);
		//printf("Grid updated %i \n",timer.getTime());	
		integrateMeasurements=0;
		printf("XXX: %i %i %i %i %s %.3f\n",len,cnt,msg->width,msg->height,msg->encoding.c_str(),depth);
	}
}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "fremengrid");
    ros::NodeHandle n;
    grid = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);
    tf_listener    = new tf::TransformListener();
    image_transport::ImageTransport imageTransporter(n); 

    ros::Time now = ros::Time(0);
    tf_listener->waitForTransform("/head_xtion_depth_optical_frame","/map",now, ros::Duration(3.0));
    ROS_INFO("Fremen grid start");

    //Subscribers:
    ros::Subscriber point_subscriber = n.subscribe<sensor_msgs::PointCloud2> ("/head_xtion/depth/points",  1000, points);
    image_transport::Subscriber image_subscriber = imageTransporter.subscribe("/head_xtion/depth/image_raw", 1, imageCallback);
    retrieve_publisher = n.advertise<visualization_msgs::Marker>("/fremenGrid/visCells", 100);

    //Services:
    ros::ServiceServer retrieve_service = n.advertiseService("/fremenGrid/visualize", visualizeGrid);
    ros::ServiceServer information_gain = n.advertiseService("/fremenGrid/entropy", estimateEntropy);
//    ros::ServiceServer add_service = n.advertiseService("/fremenGrid/measure", addView);
    ros::ServiceServer depth_service = n.advertiseService("/fremenGrid/depth", addDepth);
    ros::ServiceServer save_service = n.advertiseService("/fremenGrid/save", saveGrid);
    ros::ServiceServer load_service = n.advertiseService("/fremenGrid/load", loadGrid);

    ros::spin();
    delete tf_listener;
    delete grid;
    return 0;
}
