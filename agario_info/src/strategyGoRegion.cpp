#include "strategyGoRegion.h"

//==============================================================================
//			   Public algorithmic methods
//==============================================================================

bool StrategyGoRegion::plan(const Frame* frame, int region)
{
    auto width_game = frame->getWidth();
    auto height_game = frame->getHeight();
    auto region_with = width_game / NUM_REGIONS_X;
    auto region_height = height_game / NUM_REGIONS_Y;

    _go_to.x = region_with / 2 + region_with * (region % NUM_REGIONS_X);
    _go_to.y = region_height / 2 + region_height * (region / NUM_REGIONS_Y);

    auto limit_max = frame->getLimitMax();
    auto limit_min = frame->getLimitMin();

    _player_center = frame->getPlayer()->center;

    return (_go_to.x < limit_max.x && _go_to.y < limit_max.y &&
	    _go_to.x > limit_min.x && _go_to.y > limit_min.y);
}

void StrategyGoRegion::drawPlan(cv::Mat& img)
{
    cv::line(img, _player_center, _go_to, PLAYER_COLOR, 2, 8);
    cv::circle(img, _go_to, 4, PLAYER_COLOR, -1, 8, 0);
}
