#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>

/*delimit the OctoMap dimensions here */
#define DIM_X 54 
#define DIM_Y 42
#define DIM_Z 94 

using namespace std;
using namespace octomap;

long int times;
unsigned char *grid;
long int gridSize = DIM_X*DIM_Y*DIM_Z;
ros::Subscriber retrieveTime;
ros::Publisher markerPub;
ros::Publisher markerFuck;
std_msgs::Int32 kokot;

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

void timeCallback(const std_msgs::Int32::ConstPtr& msg)
{
	printf("Data: %i\n",msg->data);
	//init visualization markers:
	visualization_msgs::Marker markers;
	geometry_msgs::Point cubeCenter;
	std_msgs::ColorRGBA m_color;

	//set type, frame and size
	markers.header.frame_id = "/map";
	markers.header.stamp = ros::Time::now();
	markers.ns = "Markers";
	markers.action = visualization_msgs::Marker::ADD;
	markers.type = visualization_msgs::Marker::CUBE_LIST;
	markers.scale.x = resolution;
	markers.scale.y = resolution;
	markers.scale.z = resolution;
	markers.color = m_color;
	markers.points.clear();
	markers.colors.clear();

	// prepare to iterate over the entire grid
	for (int i = 0;i<gridSize;i++)
	{
		if (grid[i+msg->data*gridSize] == 1){
			cubeCenter.x = -(i%(DIM_Z+1))*resolution;
			cubeCenter.y = (i/(DIM_Y-1)/(DIM_Z+1))*resolution;
			cubeCenter.z = 3.0-((i/(DIM_Z+1))%(DIM_Y-1))*resolution;
			m_color.r = 0.0;
			m_color.b = cubeCenter.z/3.0;
			m_color.g = 1.0 - m_color.b;
			m_color.a = 1.0;
			markers.points.push_back(cubeCenter);
			markers.colors.push_back(m_color);
		}
	}
	//publish results
	markerPub.publish(markers);
}

int main(int argc,char *argv[])
{
	//*read grid
	FILE* file = fopen(argv[1],"r");
	fseek(file,0,SEEK_END);
	long int fileSize = ftell(file);
	printf("File %s of size %ld loaded.\n",argv[1],fileSize);
	long int numTimes = fileSize/gridSize;
	if (fileSize%gridSize != 0) printf("Grid file size is not divisible by %ld, file probably corrupted.\n",gridSize);
	grid = (unsigned char*) malloc(fileSize);
	fseek(file,0,0);
	fread(grid,sizeof(unsigned char),gridSize*numTimes,file);
	fclose(file);

	/*plan.prepare(200000);
	  printf("%i\n",timer.getTime());
	  timer.reset();
	  plan.prepare(200000);
	  printf("%i\n",timer.getTime());
	  return 0;*/

	ros::init(argc, argv, "froctomap");
	ros::NodeHandle n;

	//ros::NodeHandle n("~");
	n.param("resolution", resolution, 0.05);

	markerPub = n.advertise<visualization_msgs::Marker>("/froctodump/recovered", 100);
	markerFuck = n.advertise<std_msgs::Int32>("/froctodump/test", 100);
	ros::Subscriber retrieveTime = n.subscribe("/froctodump/retrieveTime", 1, timeCallback);


	ros::spin();

	return 0;
}
