#ifndef STRATEGYCHASESMALLERENEMY_H
#define STRATEGYCHASESMALLERENEMY_H

#include "strategy.h"

class StrategyChaseSmallerEnemy : public Strategy
{
public:
    bool plan(const Frame* frame);
    void drawPlan(cv::Mat& img);

private:
    cv::Point2i _player_center;

};

#endif // STRATEGYCHASESMALLERENEMY_H
