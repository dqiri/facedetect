#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <cv.h>
#include "opencv2/opencv.hpp"
#include <list>

#include <cmath>

using namespace std;
using namespace cv;

/** Function Headers */
void detectAndDisplay( Mat frame );

/** Global variables */
String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";
RNG rng(12345);

//TODO detectFaces(Mat& in, Mat& out);
//TODO will this work? 
/** @function detectAndDisplay */
void detectFace( Mat &frame )
{
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

        Mat faceROI = frame_gray( faces[i] );
        std::vector<Rect> eyes;

        //-- In each face, detect eyes
        eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

        for( size_t j = 0; j < eyes.size(); j++ )
        {
            Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
            int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
        }
    }
}

image_transport::Publisher chatter_pub;
void imageCallback(const sensor_msgs::ImageConstPtr& source)
{
    cv_bridge::CvImagePtr aa = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::MONO8);

    static cv_bridge::CvImagePtr prev = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImagePtr out = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::RGB8);

    detectFaces(out->image);

    chatter_pub.publish(out->toImageMsg());
}

/** @function main */
int main( int argc, const char** argv )
{
    CvCapture* capture;
    Mat frame;

    ros::init(argc, argv, "facedetect");

    //-- 1. Load the cascades
    if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
    if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

    //TODO: handle from here
    //-- 2. ROS from camera
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub;

    sub = it.subscribe("/camera/visibile/image", 1, imageCallback);
    chatter_pub = it.advertise("chatter", 1);
    ROS_INFO("facedetect running");
    //fl = n.advertise<sensor_msgs::Image>("removethis", 1);
    ros::spin();
    /*
    //-- 2. Read the video stream
    capture = cvCaptureFromCAM( -1 );
    if( capture )
    {
        while( true )
        {
            frame = cvQueryFrame( capture );

            //-- 3. Apply the classifier to the frame
            if( !frame.empty() )
            { detectAndDisplay( frame ); }
            else
            { printf(" --(!) No captured frame -- Break!"); break; }

            int c = waitKey(10);
            if( (char)c == 'c' ) { break; }
        }
    }
    */
    return 0;
}

