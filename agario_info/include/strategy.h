#ifndef STRATEGY_H
#define STRATEGY_H

#include "opencv2/imgproc/imgproc.hpp"
#include "frame.h"
#include <limits>

class Strategy
{
public:
    cv::Point2f getGoTo() const;

    virtual bool plan(const Frame* frame);
    virtual bool plan(const Frame* frame, int option);
    virtual void drawPlan(cv::Mat& img) = 0;

protected:
    cv::Point2f _go_to;
};

#endif  // STRATEGY_H
