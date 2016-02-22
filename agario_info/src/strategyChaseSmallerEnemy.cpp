#include "strategyChaseSmallerEnemy.h"

//==============================================================================
//			   Public algorithmic methods
//==============================================================================

bool StrategyChaseSmallerEnemy::plan(const Frame* frame)
{
    auto smaller_enemies = frame->getSmallerEnemies();
    _player_center = frame->getPlayer()->center;

    float min_dist = INT_MAX;
    std::vector<Cell>::iterator min_dist_it = smaller_enemies->end();
    for (auto it = smaller_enemies->begin(); it != smaller_enemies->end(); it++) {
	if (min_dist > it->distanceToCenter) {
	    min_dist = it->distanceToCenter;
	    min_dist_it = it;
	}
    }

    if (min_dist_it == smaller_enemies->end()) return false;

    _go_to = min_dist_it->center;
    return true;
}

void StrategyChaseSmallerEnemy::drawPlan(cv::Mat& img)
{
    cv::line(img, _player_center, _go_to, SMALLER_ENEMIES_COLOR, 2, 8);
}
