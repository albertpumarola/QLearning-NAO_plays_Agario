#ifndef STRATEGYEATPALLETS_H
#define STRATEGYEATPALLETS_H

#include "strategy.h"

class StrategyEatPellets : public Strategy
{
public:

    StrategyEatPellets();

    void setGameDimensions(int width_game, int height_game);

    bool plan(const Frame* frame);
    void drawPlan(cv::Mat& img);

private:
    cv::Point2i _player_center;
    cv::Point2i _min_center;
    cv::Point2i _max_center;

};

#endif  // STRATEGYEATPALLETS_H
