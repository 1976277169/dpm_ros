#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "image_node/imgName.h"

using namespace cv;
using namespace std;

class CImagePublisher
{
public:
	ros::NodeHandle 			m_rosHandle;
	ros::Subscriber 			m_image_name_sub;
	ros::Publisher  			m_image_status_pub;
	image_transport::ImageTransport 	m_rosImageTransport;
	image_transport::Publisher 			m_rosImagePublisher;

public:	
	CImagePublisher()
		: m_rosImageTransport( m_rosHandle )
	{
		m_image_name_sub = m_rosHandle.subscribe( "image_node/image_filename", 1000, &CImagePublisher::srvCallback, this );
		m_image_status_pub = m_rosHandle.advertise<std_msgs::String>( "image_node/srv_status", 1000 );
		m_rosImagePublisher = m_rosImageTransport.advertise( "image_node/output_image", 1 );

		std_msgs::String msg;
		msg.data = "ready";
		m_image_status_pub.publish( msg );
	}

	~CImagePublisher()
	{

	}

	void srvCallback( const std_msgs::String::ConstPtr & msg )
	{
		Mat img;
		img = imread( msg->data );

		cv_bridge::CvImage cv_image;
		cv_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
		cv_image.image = img;
		
		//imshow( "image", cv_image.image );
		//waitKey(10);
		
		m_rosImagePublisher.publish( cv_image.toImageMsg() );

		std_msgs::String pub_msg;
		pub_msg.data = "loaded";
		m_image_status_pub.publish( pub_msg );
		
		ROS_INFO( "Loaded." );
		ros::Rate loop_rate(500);
		loop_rate.sleep();
	}
};

bool loadImage( image_node::imgName::Request & req,
				image_node::imgName::Response & res );

void loadImageCallback( const std_msgs::String::ConstPtr & msg );

int main( int argc, char** argv )
{
	ros::init(argc, argv, "image_node_srv");
	
	ROS_INFO("Image_node server started.");
 	CImagePublisher publisher;
 	ros::spin();
	
	return 0;	
}

