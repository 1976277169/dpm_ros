#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CImageConventor.h"

#include "dpm_detector/detect_res.h"
#include "dpm_detector/det_list.h"
#include "dpm_detector/point.h"
#include "dpm_detector/rect.h"

using namespace std;
using namespace cv;
using namespace ros;

typedef struct DetectionResult
{
	Rect rtRoot;
	std::vector<Rect> vParts;
}_detectionResult_;

// Global variable
cv::Mat 		g_matImg;
vector<string>  g_vXMLFiles;
int 			g_nThreads;
float			g_fThreshold;

void imageCvtCallback( const sensor_msgs::ImageConstPtr& msg );
void drawResult( Mat & img, vector<DetectionResult> vResults);
void readConfigFile( string strFilename );
static void detectAndDrawObjects( Mat& image, LatentSvmDetector& detector, const vector<Scalar>& colors, float overlapThreshold, int numThreads );

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dpm_detector");

	string strCfgFilename = "/home/lgao/Workspace/ros/src/data/test.cfg";
	readConfigFile( strCfgFilename );

	g_nThreads = 12;
	g_fThreshold = 0.5f;

	ros::NodeHandle rosHandle;
	image_transport::ImageTransport 	rosImageTrans( rosHandle );
	image_transport::Subscriber		rosImageSub;

	rosImageSub = rosImageTrans.subscribe( "/image_node/output_image", 1, imageCvtCallback );

	ros::spin();

	return 0;
}

void readConfigFile( string strFilename )
{
	string line;
	ifstream file( strFilename.c_str() );

	if( file.is_open() )
	{
		int cnt = 0;
		while( getline( file, line ) )
		{
			g_vXMLFiles.push_back( line );
		}
	}
	else
	{
		ROS_ERROR("Unable to open config file.");
	}
}

void imageCvtCallback( const sensor_msgs::ImageConstPtr& msg )
{
	cv_bridge::CvImagePtr cv_ptr;

	ros::NodeHandle rosHandle;
	ros::Publisher det_pub = rosHandle.advertise<dpm_detector::det_list>( "detected_list", 1000 );

	try
    {
      cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::TYPE_8UC3 );
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    g_matImg = cv_ptr->image.clone();
    Mat detectedImg = g_matImg.clone();

    vector<string> vModelFiles;
    for( int i = 0; i < g_vXMLFiles.size(); i ++ )
    {
 	   vModelFiles.push_back( g_vXMLFiles.at(i) );
    }
    
    vector<Scalar> vColors;
    generateColors( vColors, vModelFiles.size() );

	LatentSvmDetector detector( vModelFiles );
    vector<LatentSvmDetector::ObjectDetection> detections;
    
    detector.detect( detectedImg, detections, 0.5, 8 );

    dpm_detector::det_list detected_pub;
    for( size_t i = 0; i < detections.size(); i++ )
    {
        const LatentSvmDetector::ObjectDetection& od = detections[i];
        rectangle( detectedImg, od.rect, vColors.at( od.classID ), 2 );

     	dpm_detector::detect_res res;
     	res.class_id = od.classID;
     	res.score = od.score;
     	res.rt.x = od.rect.x;
     	res.rt.y = od.rect.y;
     	res.rt.width = od.rect.width;
     	res.rt.height = od.rect.height;
     	res.cen_pt.x = ( od.rect.x + od.rect.width ) / 2;
     	res.cen_pt.y = ( od.rect.y + od.rect.height ) / 2;

     	detected_pub.det_list.push_back( res );

     	ROS_INFO_STREAM( "ClassID:" << od.classID << " Score:"<< od.score << " Centre(x,y):"
     		<< ( od.rect.x + od.rect.width ) / 2 << ", " << ( od.rect.y + od.rect.height ) / 2);
    }
    
    imshow( "Result", detectedImg );
    waitKey(20);

    det_pub.publish( detected_pub );
}