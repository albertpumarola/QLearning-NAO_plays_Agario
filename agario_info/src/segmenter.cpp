#include "segmenter.h"

//==============================================================================
//				  Constructor
//==============================================================================

void Segmenter::segment(const cv::Mat& src)
{
    cv::Mat with_borders;
    copyMakeBorder(src, with_borders, 1,1,1,1,
		   cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
    detectCircles(with_borders);
}

//==============================================================================
//				    Getters
//==============================================================================

std::vector<float> Segmenter::getRadius() const { return _radius; }

std::vector<cv::Point2f> Segmenter::getCenter() const { return _center; }

std::vector<cv::Vec3f> Segmenter::getCircles() const { return _circles; }

std::vector<std::vector<cv::Point>> Segmenter::getPolyContour() const
{
    return _contours_poly;
}

cv::Mat Segmenter::getProcessedImg() const { return _src_gray; }

//==============================================================================
//			  Private algorithmic methods
//==============================================================================

//TODO improve detection. Problems with superpositions and occlusions
void Segmenter::detectCircles(const cv::Mat& src)
{
    /// Convert it to gray
    cv::cvtColor(src, _src_gray, CV_BGR2GRAY);

    /// Reduce the noise so we avoid false circle detection
    cv::GaussianBlur(_src_gray, _src_gray,
		     cv::Size(GAUSSIAN_BLUR_SIZE, GAUSSIAN_BLUR_SIZE), 2, 2);

    cv::threshold(_src_gray, _src_gray, THRESHOLD_VALUE, 255,
		  cv::THRESH_BINARY);

    // cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
    // cv::Size( 2*EROSION_SIZE + 1, 2*EROSION_SIZE+1 ),
    // cv::Point( EROSION_SIZE, EROSION_SIZE ) );

    ///// Apply the erosion operation
    // cv::dilate( _src_gray, _src_gray, element );
    // cv::erode( _src_gray, _src_gray, element );

    /// Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat extended(_src_gray.size() + cv::Size(2, 2), _src_gray.type());
    cv::findContours(_src_gray, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,
		     cv::Point(0, 0));

    /// Approximate contours to polygons + get bounding rects and circles
    _contours_poly = std::vector<std::vector<cv::Point>>(contours.size());
    _center = std::vector<cv::Point2f>(contours.size());
    _radius = std::vector<float>(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
	cv::approxPolyDP(cv::Mat(contours[i]), _contours_poly[i], 1, true);
	cv::minEnclosingCircle((cv::Mat)_contours_poly[i], _center[i],
			       _radius[i]);
    }

     //filetr border centers
     for (size_t i = 0; i < _center.size(); ++i) {
     	 if(_center[i].x == 0 || _center[i].x == src.cols || _center[i].y == 0 || _center[i].y == src.rows){
     	     _center.erase(_center.begin()+i);
     	     _contours_poly.erase(_contours_poly.begin()+i);
     	     _radius.erase(_radius.begin()+i);
	 }
    }

    /// Apply the Hough Transform to find the circles
    // cv::HoughCircles( _src_gray, _circles, CV_HOUGH_GRADIENT,
    // HOUGH_INV_RATIO_RESOL, HOUGH_MIN_DIST, HOUGH_UPPER_THRESH,
    // HOUGH_CENTER_THRESH, HOUGH_MIN_RADIUS, HOUGH_MAX_RADIUS);
}

