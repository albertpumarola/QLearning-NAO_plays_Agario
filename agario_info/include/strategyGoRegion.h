#ifndef STRATEGYGOREGION_H
#define STRATEGYGOREGION_H

#include "strategy.h"
#include "globals.h"

class StrategyGoRegion : public Strategy
{
public:
    bool plan(const Frame* frame, int region);
    void drawPlan(cv::Mat& img);
private:
    cv::Point2i _player_center;
};

#endif // STRATEGYGOREGION_H
