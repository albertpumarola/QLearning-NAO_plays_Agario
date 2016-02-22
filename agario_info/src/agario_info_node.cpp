#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "agario_mouse/Mouse.h"
#include "agario_sele/Notify.h"
#include <dynamic_reconfigure/server.h>
#include <agario_info/dynparamsConfig.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <cmath>
#include "globals.h"
#include "segmenter.h"
#include "frame.h"
#include "decisionMaker.h"
#include "imageFromDisplay.h"

/// Has a new image arrived?
bool new_image = false;
bool is_alive = false;
bool train_mode;

void getParameters(ros::NodeHandle& nh)
{
    // get process configuration file path
    if (!nh.getParam("agario_info/img_path", IMG_PATH)) {
	ROS_ERROR("Failed to get param '/agario_info/img_path'");
	ros::shutdown();
    }

    // get Q matrix file path
    if (!nh.getParam("agario_info/Q_path", Q_FILE)) {
	ROS_ERROR("Failed to get param '/agario_info/Q_path'");
	ros::shutdown();
    }

    if (!nh.getParam("agario_info/Q_save_path", Q_SAVE_PATH)) {
	ROS_ERROR("Failed to get param '/agario_info/Q_save_path'");
	ros::shutdown();
    }

    if (!nh.getParam("agario_info/train_mode", train_mode)) {
	ROS_ERROR("Failed to get param '/agario_info/train_mode'");
	ros::shutdown();
    }
}

void callbackDynConf(agario_info::dynparamsConfig& config, uint32_t level)
{
    GAUSSIAN_BLUR_SIZE = std::round(config.gaussian_size);
    THRESHOLD_VALUE = config.threshold_value;
    EROSION_SIZE = config.erosion_size;
    CELL_MAX_RADIUS = config.cell_max_radius;
    CELL_MIN_RADIUS = config.cell_min_radius;
    CELL_CENTER_MIN_DIST = config.cell_center_min_dist;
    PELLET_RADIUS_RANGE = config.pellet_radius_range;
    MAX_SHOOT_DIST = config.max_shoot_dist;
    EVADE_SCAN_DISTANCE = config.evade_scan_distance;

    NUM_RAYS = config.num_rays;
    RAYS_RANGE = config.rays_range;

    NUM_PELLETS_REGIONS_X = config.num_pellets_regions_x;
    NUM_PELLETS_REGIONS_Y = config.num_pellets_regions_y;

    IR_MAX_X = config.ir_max_x;
    IR_MAX_Y = config.ir_max_y;
    IR_MIN_X = config.ir_min_x;
    IR_MIN_Y = config.ir_min_y;

    MOUSE_CALIBRATE_X = config.mouse_calibrate_x;
    MOUSE_CALIBRATE_Y = config.mouse_calibrate_y;

    RADIUS_ADDED_WHEN_EATEN_PELLET = config.radius_added_when_eaten_pellet;
}

void has_died() { is_alive = false; }

void has_respawned() { is_alive = true; }

void moveMouse(ros::ServiceClient& client, const cv::Point2f& goal)
{
    agario_mouse::Mouse srv;
    srv.request.x = (int)goal.x + MOUSE_CALIBRATE_X;
    srv.request.y = (int)goal.y + MOUSE_CALIBRATE_Y;
    client.call(srv);
}

bool cb(agario_sele::Notify::Request& req, agario_sele::Notify::Response& res)
{
    if (req.is)
	has_respawned();
    else
	has_died();
    res.ready = 1;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "agario_indo_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    image_transport::ImageTransport image_transport(n);

    // subscribers
    image_transport::Publisher image_pub =
	image_transport.advertise("video_output", 1);

    // service
    ros::ServiceClient client_mouse =
	n.serviceClient<agario_mouse::Mouse>("agario_mouse");
    ros::ServiceServer notify_death =
	n.advertiseService("agario_info_switch", cb);

    dynamic_reconfigure::Server<agario_info::dynparamsConfig> server;
    dynamic_reconfigure::Server<agario_info::dynparamsConfig>::CallbackType f;
    f = boost::bind(&callbackDynConf, _1, _2);
    server.setCallback(f);

    // get node parameters
    getParameters(n);

    cv::namedWindow("Results", CV_WINDOW_NORMAL);
    ImageFromDisplay image_from_display;
    DecisionMaker decision_maker(train_mode);
    float previous_player_radious = 0;
    while (ros::ok()) {
	if (is_alive) {
	    auto frame_img = image_from_display.getImage();

	    if (frame_img.rows && frame_img.cols) {
		Segmenter segmenter;
		segmenter.segment(frame_img);

		Frame frame(frame_img, segmenter.getCenter(),
			    segmenter.getRadius(), segmenter.getPolyContour(),
			    frame_img.rows, frame_img.cols, is_alive,
			    previous_player_radious);

		decision_maker.execute(&frame);

		auto goal = decision_maker.getGoal();
		moveMouse(client_mouse, goal);

		previous_player_radious = frame.getPlayer()->radius;

		if (DISPLAY) {
		    frame.display(frame_img);
		    decision_maker.drawPelletsRegions(frame_img);
		    decision_maker.drawRays(frame_img);
		    decision_maker.drawAllPlans(frame_img);
		    // decision_maker.drawPlan(frame_img);

		    if (RECORD) {
			sensor_msgs::ImagePtr img_msg =
			    cv_bridge::CvImage(std_msgs::Header(), "bgr8",
					       frame_img).toImageMsg();
			image_pub.publish(img_msg);
		    }

		    /// Show your results
		    cv::imshow("Results", frame_img);
		    cv::waitKey(5);
		}
	    }

	} else {
	    if (RECORD) {
		cv::Mat null;
		sensor_msgs::ImagePtr img_msg =
		    cv_bridge::CvImage(std_msgs::Header(), "bgr8", null)
			.toImageMsg();
		image_pub.publish(img_msg);
	    }
	}
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}
