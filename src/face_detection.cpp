#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

/** Global variables */
static const std::string OPENCV_WINDOW = "Image window";
Mat frame;
String face_cascade_name = "/home/linaro/opencv/data/haarcascades/haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "/home/linaro/opencv/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("camera/visible/image", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("image", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void detectAndDisplay( Mat& ret, const Scalar& color )
    {
        std::vector<Rect> faces;
        Mat frame_gray;
        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        //equalizeHist( frame_gray, frame_gray );

        //-- Detect faces
        face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

        if ( faces.size() == 0 ) ROS_INFO("No face is detected!!!\n");

        for( size_t i = 0; i < faces.size(); i++ )
        {
            //Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
            //ellipse( ret, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, color, 4, 8, 0 );
            rectangle( ret, faces[i].tl(), faces[i].br(), color, 2, 8, 0 );

            Mat faceROI = frame_gray( faces[i] );
            std::vector<Rect> eyes;

            //-- In each face, detect eyes
            eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

            for( size_t j = 0; j < eyes.size(); j++ )
            {
                Point first( faces[i].x + eyes[j].x, faces[i].y + eyes[j].y);
                Point last( faces[i].x + eyes[j].x + eyes[j].width, faces[i].y + eyes[j].y + eyes[j].height);
                rectangle(ret,first, last, color, 2, 8, 0);
                //Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
                //int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
                //circle( ret, center, radius, color, 4, 8, 0 );
            }
        }
        //-- Show what you got
        //imshow( window_name, frame_gray );
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat cflow, ret;//, flow2;
        frame = cv_ptr->image;
        frame.copyTo(ret);
        //-- 1. Load the cascades
        if( !face_cascade.load( face_cascade_name ) ){ ROS_ERROR("--(!)Error loading\n"); return; };
        if( !eyes_cascade.load( eyes_cascade_name ) ){ ROS_ERROR("--(!)Error loading\n"); return; };

        //-- 2. Apply the classifier to the frame
        if( !frame.empty() )
        {
            detectAndDisplay( ret, CV_RGB( 0,0,255 ) );
        }
        else
        { ROS_ERROR(" --(!) No captured frame -- Break!"); return;}

        cv_bridge::CvImage out;
        out.header = msg->header;
        out.encoding = msg->encoding;
        out.image = ret;
        image_pub_.publish(out.toImageMsg());//cv_ptr->toImageMsg());
    }
};

int main( int argc, char * argv[] )
{
    ros::init(argc, argv, "face_detection");
    ros::NodeHandle node;
    ImageConverter ic;

    while ( ros::ok() )
    {
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}
