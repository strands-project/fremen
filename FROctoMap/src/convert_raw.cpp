#include <ros/ros.h>
#include <sensor_msgs/Image.h>			//Depth Image
#include <sensor_msgs/CameraInfo.h>		//CameraInfo

std::string filename, directory;

float D[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float K[] = {525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0};
float R[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
float P[] = {525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_data");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  
  //Name of the file received as parameter
  //nh.param<std::string>("filename", filename, "week.3d");
  //nh.param<std::string>("directory", directory, "/home/joao/catkin_ws/");
  
//  directory += filename;
  
 // ROS_INFO("%s", directory.c_str());
  
  //Opening the file for reading
  FILE *file3D = fopen(argv[1],"r");
  
  if (file3D == NULL)
  {
    ROS_INFO("Error openning the file!");
    return(0);
  }
  
  //Publisher (Depth Image)
  ros::Publisher depth_image_pub = n.advertise<sensor_msgs::Image>("/camera/depth/image_raw",1000);
  ros::Publisher camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info",1000);
  
  //Depth Image Structure:
  sensor_msgs::Image depth_image;
  depth_image.header.frame_id = "/depth_optical_frame";
  depth_image.height = 240;
  depth_image.width = 320;
  depth_image.encoding = "16UC1";
  depth_image.is_bigendian = 1;
  depth_image.step = 320 * 2;
  
  //Camera Info Structure:
  sensor_msgs::CameraInfo camera_info;
  camera_info.header.frame_id = "/depth_optical_frame";
  camera_info.height = 240;
  camera_info.width = 320;
  camera_info.distortion_model = "plumb_bob";
  
  camera_info.D.insert(camera_info.D.end(), D, D + 5);

  for(int i = 0; i < 9; i++)
  {
    camera_info.K[i] = K[i];
    camera_info.R[i] = R[i];
  }
  
  for(int i = 0; i < 12; i++)
     camera_info.P[i] = P[i];
//   memcpy(&camera_info.R, R, 9 * sizeof *R);
//   memcpy(&camera_info.P, P, 12 * sizeof *P)*;

  
  camera_info.binning_x = 0;
  camera_info.binning_y = 0;
  camera_info.roi.x_offset = 0;
  camera_info.roi.y_offset = 0;
  camera_info.roi.height = 0;
  camera_info.roi.width = 0;
  camera_info.roi.do_rectify = false;
  
  
  //Auxiliary Variables:
  long long data_timestamp;
  
  short int *buffer;
  
  unsigned int DataSize = 320*240;
  
  depth_image.data.resize(depth_image.step * depth_image.height);

  ros::Rate r(7);
  usleep(2000000); 
  
  ROS_INFO("Publishing data...");
  int a = 0; 
  buffer = (short int*) malloc (sizeof(short int)*DataSize);
  while(ros::ok() && feof(file3D)==0){
	  //Read the timestamp:
	  if (fread(&data_timestamp,8,1,file3D)==1)
	  {
		  //Read the depth data:
		  fread(buffer,DataSize*2,1,file3D);

		  for(int i = 0; i < DataSize; i++)
		  {
			  depth_image.data[2*i] = (buffer[i])%256;
			  depth_image.data[2*i+1] = (buffer[i])/256;
		  }

		  //ros::Time test(data_timestamp/1000000,data_timestamp%1000000);
		  ros::Time test = ros::Time::now();
		  depth_image.header.stamp = test; //ros::Time(,):
		  camera_info.header.stamp = test;

		  camera_info_pub.publish(camera_info);
		  depth_image_pub.publish(depth_image);
		  usleep(500000); 
		  //r.sleep();
	  }
	  a++;
  }
  free(buffer);
  fclose(file3D);

  return(0);

}
