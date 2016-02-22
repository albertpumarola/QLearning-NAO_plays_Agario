#ifndef GLOBALS_H
#define GLOBALS_H

#include <string>
#include "opencv2/imgproc/imgproc.hpp"

extern int GAUSSIAN_BLUR_SIZE;
extern std::string IMG_PATH;
extern int HOUGH_INV_RATIO_RESOL;
extern int HOUGH_MIN_DIST;
extern int HOUGH_UPPER_THRESH;
extern int HOUGH_CENTER_THRESH;
extern int HOUGH_MIN_RADIUS;
extern int HOUGH_MAX_RADIUS;
extern double THRESHOLD_VALUE;
extern int EROSION_SIZE;
extern double CELL_MAX_RADIUS;
extern double CELL_MIN_RADIUS;
extern double CELL_CENTER_MIN_DIST;
extern double PELLET_RADIUS_RANGE;
extern double MAX_SHOOT_DIST;
extern double EVADE_SCAN_DISTANCE;
extern int NUM_RAYS;
extern int RAYS_RANGE;
extern int NUM_PELLETS_REGIONS_X;
extern int NUM_PELLETS_REGIONS_Y;
extern int NUM_REGIONS_X;
extern int NUM_REGIONS_Y;

extern int IR_MAX_X;
extern int IR_MAX_Y;
extern int IR_MIN_X;
extern int IR_MIN_Y;

extern int MOUSE_CALIBRATE_X;
extern int MOUSE_CALIBRATE_Y;

extern float RADIUS_ADDED_WHEN_EATEN_PELLET;

extern std::string Q_FILE;
extern std::string Q_SAVE_PATH;

static const int R_HAS_SURVIVED = 1;
static const int R_HAS_DIED = -100;
static const int R_HAS_EATEN_PELLET = 5;
static const int R_HAS_EATEN_CELL = 10;

static const int NUM_ACTIONS = 3 + NUM_REGIONS_X * NUM_REGIONS_Y;
static const long Q_NUM_STATES =
    std::pow(2, NUM_RAYS) * NUM_REGIONS_X * NUM_REGIONS_Y;
static const double Q_GAMMA = 0.5;
static const double Q_EXPLORATION_FACTOR = 0.2;
static const int SAVE_Q_EVERY = 1;  // save Q every x games

static const cv::Scalar BIGGER_ENEMIES_COLOR = cv::Scalar(0, 0, 255);
static const cv::Scalar SMALLER_ENEMIES_COLOR = cv::Scalar(0, 255, 0);
static const cv::Scalar PELLET_COLOR = cv::Scalar(255, 255, 0);
static const cv::Scalar VIRUS_COLOR = cv::Scalar(255, 0, 0);
static const cv::Scalar PLAYER_COLOR = cv::Scalar(0, 0, 0);
static const cv::Scalar LIMIT_COLOR = cv::Scalar(0, 0, 0);
static const cv::Scalar NULL_COLOR = cv::Scalar(255, 255, 255);
static const cv::Scalar DANGEROUS_COLOR = cv::Scalar(0, 0, 255);
static const cv::Scalar SAFE_COLOR = cv::Scalar(0, 255, 0);

//action num
static const int EAT_PELLET = 0;
static const int CHASE_ENEMY = 1;
static const int EVADE = 2;
static const int GO_REGION = 3;


static const bool DISPLAY = false;
static const bool RECORD = false;

#endif  // GLOBALS_H
