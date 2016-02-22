#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "agario_sele/Notify.h"
#include <string>
#include <iostream>

bool publish = false;

bool cb(agario_sele::Notify::Request& req, agario_sele::Notify::Response& res)
{
    publish = req.is;
    res.ready = 1;
    return true;
}

void ImageFromDisplay(std::vector<uint8_t>& Pixels, int& Width, int& Height,
		      int& BitsPerPixel)
{
    Display* display = XOpenDisplay(nullptr);
    Window root = DefaultRootWindow(display);

    XWindowAttributes attributes = {0};
    XGetWindowAttributes(display, root, &attributes);

    Width = attributes.width;
    Height = attributes.height;

    XImage* img =
	XGetImage(display, root, 0, 0, Width, Height, AllPlanes, ZPixmap);
    BitsPerPixel = img->bits_per_pixel;
    Pixels.resize(Width * Height * 4);

    memcpy(&Pixels[0], img->data, Pixels.size());

    XFree(img);
    XCloseDisplay(display);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stream_screen_img_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    int Width = 0;
    int Height = 0;
    int Bpp = 0;
    std::vector<std::uint8_t> Pixels;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("video_output", 1);

    ros::ServiceServer service =
	nh.advertiseService("stream_screen_img_switch", cb);

    // object to store the output video msgs
    sensor_msgs::ImagePtr msg;

    while (ros::ok()) {
	if (publish) {
	    // Get a new frame from camera
	    ImageFromDisplay(Pixels, Width, Height, Bpp);

	    if (Width && Height) {
		cv::Mat img =
		    cv::Mat(Height, Width, CV_8UC4,
			    &Pixels[0]);  // Mat(Size(Height, Width), Bpp > 24 ?
					  // CV_8UC4 : CV_8UC3, &Pixels[0]);

		// publish msg
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", img)
			  .toImageMsg();
		pub.publish(msg);
	    }
	}
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}
