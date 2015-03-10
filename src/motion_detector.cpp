#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <assignment_5/Mode.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>
#include <cmath>

using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";
int mode;
cv::Mat prevgray, gray, flow, frame, flow2, fgMaskMOG2;
Ptr<BackgroundSubtractorMOG2> pMOG2;
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

    void drawOptFlowMap(const Mat& flow, Mat& cflowmap, Mat& ret, const Scalar& color)
    {
        Mat threshold_output;
        vector<vector<Point> > contours;
        Mat flow1d;
        Mat goodFlow;
        if (mode == 1)
        {
            cflowmap.copyTo(goodFlow);
            /* Obtain magnitudes from flow */
            for(int y = 0; y < goodFlow.rows; y += 1)
                for(int x = 0; x < goodFlow.cols; x += 1)
                {
                    const Point2f& fxy = flow.at<Point2f>(y, x);
                    goodFlow.data[goodFlow.channels()*goodFlow.cols*y + x + 0] = (int)(sqrt(fxy.x*fxy.x + fxy.y*fxy.y));

                    /* if (15.0 < (value = (sqrt(fxy.x*fxy.x + fxy.y*fxy.y))))
                       {

                       goodFlow.data[goodFlow.channels()*goodFlow.cols*y + x + 0] = 255;

                       }   //cflowmap.at<uchar>(y,x,1) = 255;
                       else
                       {

                       goodFlow.data[goodFlow.channels()*goodFlow.cols*y + x + 0] =0 ;
                       }*/
                    //cflowmap.at<uchar>(y,x,1) = 0;
                }
            //ROS_INFO("MAX: %f", max);
            /*     Mat element = getStructuringElement( erosion_type,
                   Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                   Point( erosion_size, erosion_size ) );
                   erode(goodFlow, flow1d, element);*/
            //goodFlow = flow1d;
            //flow1d.convertTo(goodFlow, CV_8U);
            /* for(int y = 0; y < flow1d.rows*flow1d.cols; y += 1)
               {
               if ( goodFlow.at<float>(y) )
               ROS_INFO("Value at %d is %f\n", y,goodFlow.at<float>(y) );
               }
               exit(1);*/

            /// Detect edges using Threshold
            blur(goodFlow, goodFlow, Size(25,25));
            threshold( goodFlow, threshold_output, 15, 255, THRESH_BINARY );
            GaussianBlur(threshold_output, threshold_output, Size(35,35), 200, 0);
            erode(threshold_output, flow1d, Mat(), Point(-1, -1), 10, 1, 1);
            dilate(flow1d, threshold_output, Mat(), Point(-1, -1), 20, 1, 1);
            GaussianBlur(threshold_output, threshold_output, Size(15,15), 200, 0);
            dilate(threshold_output, flow1d, Mat(), Point(-1, -1), 20, 1, 1);
            ///cvtColor(threshold_output, threshold_output, CV_BGR2GRAY);
            /// Find contours
            //imshow(OPENCV_WINDOW, flow1d);
            //waitKey(3);
        }
        else if (mode == 2)
        {
            cflowmap.copyTo(goodFlow);
            blur(goodFlow, goodFlow, Size(15,15));
            threshold( goodFlow, threshold_output, 15, 255, THRESH_BINARY );
//            imshow(OPENCV_WINDOW, threshold_output);
//            waitKey(3);
            GaussianBlur(threshold_output, threshold_output, Size(5,5), 25, 0);
            erode(threshold_output, flow1d, Mat(), Point(-1, -1), 10, 1, 1);
            dilate(flow1d, threshold_output, Mat(), Point(-1, -1), 20, 1, 1);
            GaussianBlur(threshold_output, threshold_output, Size(15,15), 200, 0);
            dilate(threshold_output, flow1d, Mat(), Point(-1, -1), 20, 1, 1);

        }
        findContours( flow1d , contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );
        vector<Point2f>center( contours.size() );

        for( int i = 0; i < contours.size(); i++ )
        {
            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        }
            cvtColor(cflowmap, flow2, CV_GRAY2BGR);
//        Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {
            rectangle( ret, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        }

        /*
           int count = 0;
           Point2f start;
        Point startpoint;
        for(int y = 0; y < cflowmap.rows; y += step)
            for(int x = 0; x < cflowmap.cols; x += step)
            {
                const Point2f& fxy = flow.at<Point2f>(y, x);
                start = fxy;
                if( (fxy.x > 1 || fxy.y > 1) && startpoint.x == -10 )
                {
                    startpoint = Point(x,y);
                }
                else if (!(abs(fxy.x) > 1 || abs(fxy.y) > 1))
                {
                    rectangle(cflowmap, Point(startpoint.x, startpoint.y), Point(cvRound(fxy.x), cvRound(fxy.y)), color);
                    ROS_INFO("end of square. start.x: %d, fxy.x: %f", startpoint.x, fxy.x);
                    startpoint.x = -10;
                }
                */


                /*  const Point2f& fxy = flow.at<Point2f>(y, x);
                    line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                    color);
                    circle(cflowmap, Point(x,y), 2, color, -1);
                    }*/
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

        // Draw an example circle on the video stream

        Mat cflow, ret;//, flow2;
        frame = cv_ptr->image;
        frame.copyTo(ret);
        if (mode == 1)
        {
            cvtColor(frame, gray, CV_BGR2GRAY);
            if( prevgray.data )
            {
                calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

                prevgray.copyTo(cflow);
/*
std::cerr << "image chans : " << cflow.channels() << std::endl;
                cvtColor(flow, flow2,
std::cerr << "image chans : " << flow2.channels() << std::endl;
*/
                blur(cflow, cflow, Size(3,3));
                drawOptFlowMap(flow, cflow, ret, CV_RGB(0, 0, 255));

                //imshow(OPENCV_WINDOW, cflow);
                //ROS_INFO("prevgray previously set");
            }
            std::swap(prevgray, gray);
            //cvtColor(cflow, flow2, CV_GRAY2BGR);
        }

        //call MOG
        else if (mode == 2)
        {
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            frame = cv_ptr->image;
            //cvtColor(frame, gray, CV_BGR2GRAY);

            //frame.convertTo(gray, CV_8UC1);
            //pMOG2 = BackgroundSubtractorMOG2();
            pMOG2->operator()(frame, fgMaskMOG2);
            //fgMaskMOG2.copyTo(ret);
            drawOptFlowMap(flow, fgMaskMOG2, ret, CV_RGB(0, 0, 255));

        }
        cv_bridge::CvImage out;
        out.header = msg->header;
        out.encoding = msg->encoding;
        out.image = ret;
        image_pub_.publish(out.toImageMsg());//cv_ptr->toImageMsg());

        // Update GUI Window
        //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //cv::waitKey(3);

        // Output modified video stream

    }
};

bool chooseMode(assignment_5::Mode::Request  &req,
         assignment_5::Mode::Response &res)
{
    if ( req.mode == 1 )
    {
        mode = 1;
    }
    else if ( req.mode == 2 )
    {
        mode = 2;
    }
    return true;
}

int main( int argc, char * argv[] )
{
    ros::init(argc, argv, "motion_detector");
    ros::NodeHandle node;
    ImageConverter ic;
    ros::ServiceServer service = node.advertiseService("Mode", chooseMode);
    pMOG2 = new BackgroundSubtractorMOG2();

    while ( ros::ok() )
    {
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}
