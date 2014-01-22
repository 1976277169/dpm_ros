#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_node/imgName.h"
#include <iostream>

using namespace std;
using namespace ros;

// Global variables
int g_nServerStatus;

void statusCallback( const std_msgs::String::ConstPtr & msg );

int main( int argc, char ** argv )
{
	g_nServerStatus = 1;

	ros::init( argc, argv, "image_node_clident" );

	if( argc != 2 )
	{
		ROS_INFO("usage: image_node_client image filename.");;
		return 1;
	}

	ros::NodeHandle rosHandle;
	ros::Subscriber image_node_status = rosHandle.subscribe<std_msgs::String>( "image_node/srv_status", 1000, statusCallback );
	ros::Publisher image_filename_pub = rosHandle.advertise<std_msgs::String>( "image_node/image_filename", 1000 );
	
	string strImgName( argv[1] );
	ROS_INFO("Image filename: %s", strImgName.c_str());

	ros::Rate loop_rate(10);
	while ( ros::ok() && ( g_nServerStatus == 1 ) )
	{
		std_msgs::String msg;
		msg.data = strImgName;

		image_filename_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();	
	}

	return 0;
}

void statusCallback( const std_msgs::String::ConstPtr & msg )
{
	if( msg->data == "ready" )
	{
		g_nServerStatus = 1;
	}
	else
	{
		g_nServerStatus = 0;
	}
}