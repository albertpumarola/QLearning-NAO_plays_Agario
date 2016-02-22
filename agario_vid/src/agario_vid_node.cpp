#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxcore.h>
#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

static const int VIDEO_WIDTH = 1856;
static const int VIDEO_HEIGHT = 1056;
static const int FPS = 25;

// Object to store the new received image
cv_bridge::CvImagePtr raw_frame;
bool new_image = false;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    try {
	raw_frame =
	    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
    }
    new_image = true;
}

void wrtVid(const cv::Mat& matImage, int wt, int ht,
	    CvVideoWriter* writer)  // matImage la imatge
{
    IplImage ipl_img = matImage;
    cvWriteFrame(writer, &ipl_img);  // add the frame to the file
}

void playVid(const std::string& video_path_out)
{
    cv::VideoCapture capture(video_path_out);
    cv::Mat edges;
    int askFileTypeBox = 0;  //-1 is show box of codec
    int Color = 1;
    cv::Size S = cv::Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH),
		  (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    for (;;) {
	cv::Mat frame;
	capture >> frame;  // get a new frame from camera
	if (frame.rows != 0 && frame.cols != 0) {
	    cv::imshow("frame", frame);
	}
	if (cv::waitKey(30) >= 0) break;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "agario_vid_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(FPS);

    image_transport::ImageTransport image_transport(n);

    // subscribers
    image_transport::Subscriber image_sub =
	image_transport.subscribe("video_input", 1, &imageCb);

    // get video path
    std::string video_path_out;
    if (!n.getParam("agario_vid/video_path_out", video_path_out)) {
	ROS_ERROR("Path error agario_vid/video_path_out");
	return 1;
    }
    const char* cstr = video_path_out.c_str();

    CvSize size;
    size.width = VIDEO_WIDTH;
    size.height = VIDEO_HEIGHT;

    CvVideoWriter* writer =
	cvCreateVideoWriter(cstr, CV_FOURCC('M', 'J', 'P', 'G'), FPS, size);
    ROS_INFO("Video created");

    while (ros::ok()) {
	if (new_image) {

	    if (raw_frame->image.rows == 0 || raw_frame->image.cols == 0) {
		ROS_INFO("end detected");
		cvReleaseVideoWriter(&writer);
	    	break;

	    } else {
		wrtVid(raw_frame->image, size.width, size.height, writer);
	    }
	    new_image = false;
	}
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}
