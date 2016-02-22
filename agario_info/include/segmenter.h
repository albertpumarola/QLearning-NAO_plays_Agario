#ifndef SEGMENTER_H
#define SEGMENTER_H

#include "globals.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <cmath>

class Segmenter
{
public:
    void segment(const cv::Mat& src);

    std::vector<cv::Vec3f> getCircles() const;
    cv::Mat getProcessedImg() const;
    std::vector<float> getRadius() const;
    std::vector<cv::Point2f> getCenter() const;
    std::vector<std::vector<cv::Point> > getPolyContour() const;


private:
    std::vector<cv::Vec3f> _circles;
    cv::Mat _src_gray;
    std::vector<cv::Point2f> _center;
    std::vector<float> _radius;
    std::vector<std::vector<cv::Point> > _contours_poly;

    void detectCircles(const cv::Mat& src);

};

#endif // SEGMENTER_H
