#include "ros/ros.h"
#include <ctime>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>
#include <std_msgs/Int8.h>

#define COOLDOWN_TIMER 10
#define COUNTDOWN_TIMER 2
#define IMAGE_DIRECTORY "/home/linaro/catkin_ws/src/selfiebot/images/"

using namespace std;
using namespace cv;

/** Global variables */
static const std::string OPENCV_WINDOW = "Image window";
static time_t timer = 0;
int cooldown;
int ledValue = 0;
Mat frame;
String face_cascade_name = "/home/linaro/catkin_ws/src/selfiebot/haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;
string window_name = "Capture - Face detection";
ros::Publisher pub;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    
    public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("camera/visible/image", 1,
                &ImageConverter::imageCb, this);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    bool hasFace()
    {
        std::vector<Rect> faces;
        Mat frame_gray;
        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        equalizeHist( frame_gray, frame_gray );
        face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
        return faces.size() > 0;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {

        std_msgs::Int8 value;
        value.data = 0;
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

        Mat outputImage;
        frame = cv_ptr->image;
        //-- 1. Load the cascades
        if( !face_cascade.load( face_cascade_name ) ){ ROS_ERROR("--(!)Error loading\n"); return; };

        //-- 2. Apply the classifier to the frame
        if( !frame.empty() )
        {
            /* ON COOLDOWN */
            if ( cooldown )
            {
                if ( difftime(time(NULL), timer) > 5 )
                {
                   /* value.data = 2;
                    pub.publish(value);
                }
                else
                {*/
                    value.data = 0;
                    pub.publish(value);
                //}
}
                if ( difftime(time(NULL), timer) >= COOLDOWN_TIMER )
                {
                    cooldown = 0;
                }
            }
            /* FACE FOUND */
            else
            {
              value.data = 2;
              pub.publish(value);
            if ( hasFace() )
            {
                //value.data = 2;
                //pub.publish(value);
                if ( !timer )
                {
                    time(&timer);
                }
                else if ( difftime(time(NULL), timer) >= COUNTDOWN_TIMER )
                {
                    time(&timer);
                    char outputfilename[80];
                    char timestring[30];
                    strftime(timestring, 30, "%m%d%y_%H%M%S", localtime(&timer));
                    snprintf(outputfilename, 80, "%simage_%s.jpg", IMAGE_DIRECTORY, timestring);
                    cvtColor( frame, outputImage, CV_BGR2RGB );
                    imwrite(outputfilename, outputImage);
                    ROS_INFO("Your picture has been taken =P"); 
                    value.data = 1;
                    pub.publish(value);
                    cooldown = 1;
                }
            }
            /* FACE NOT FOUND */
            else
            {
                timer = 0;
                //ledValue = 0;
                //value.data = 0;
                //pub.publish(value);
            }
}
        }
        else
        { ROS_ERROR(" --(!) No captured frame -- Break!"); return;}
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "selfiebot");
    ros::NodeHandle n;
    ImageConverter ic;
    pub = n.advertise<std_msgs::Int8>("led_value", 1000);
    while ( ros::ok() )
    {
        //value.data = ledValue;
        //pub.publish(value);
        
        ros::spinOnce();
    }
        std_msgs::Int8 value;
        value.data = 0;
        pub.publish(value);
}
