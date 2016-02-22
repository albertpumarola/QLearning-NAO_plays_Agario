#include "strategyEatPellets.h"

//==============================================================================
//				  Constructors
//==============================================================================

StrategyEatPellets::StrategyEatPellets()
    : _min_center(-1, -1), _max_center(-1, -1)
{
}

//==============================================================================
//				    Setters
//==============================================================================

void StrategyEatPellets::setGameDimensions(int width_game, int height_game)
{
    auto region_with = width_game / NUM_REGIONS_X;
    auto region_height = height_game / NUM_REGIONS_Y;
    _min_center.x = region_with;
    _min_center.y = region_height;
    _max_center.x = 2 * region_with;
    _max_center.y = 2 * region_height;
}

//==============================================================================
//			   Public algorithmic methods
//==============================================================================

bool StrategyEatPellets::plan(const Frame* frame)
{
    // dimensions have been initialized?
    if (_min_center.x == -1)
	setGameDimensions(frame->getWidth(), frame->getHeight());

    auto pellets = frame->getPellets();
    _player_center = frame->getPlayer()->center;
    auto player_r = frame->getPlayer()->radius;

    float min_dist = INT_MAX;
    std::vector<Cell>::iterator min_dist_it = pellets->end();
    for (auto it = pellets->begin(); it != pellets->end(); it++) {
	if (it->center.x < _max_center.x && it->center.x > _min_center.x &&
	    it->center.y<_max_center.y & it->center.y> _min_center.y) {
	    if (min_dist > it->distanceToCenter &&
		it->distanceToCenter > player_r) {
		min_dist = it->distanceToCenter;
		min_dist_it = it;
	    }
	}
    }

    if (min_dist_it == pellets->end()) return false;

    _go_to = min_dist_it->center;
    return true;
}

void StrategyEatPellets::drawPlan(cv::Mat& img)
{
    cv::line(img, _player_center, _go_to, PELLET_COLOR, 2, 8);
}
