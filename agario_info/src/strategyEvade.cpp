#include "strategyEvade.h"

//==============================================================================
//			   Public algorithmic methods
//==============================================================================

bool StrategyEvade::plan(const Frame* frame)
{
    auto bigger_enemies = frame->getBiggerEnemies();
    if (bigger_enemies->size() == 0) return false;

    _player_center = frame->getPlayer()->center;

    std::priority_queue<cv::Point2f, std::vector<cv::Point2f>,
			enemies_comparasion> ordered_enemies(_player_center);

    for (auto it = bigger_enemies->begin(); it != bigger_enemies->end(); it++) {
	if (it->distanceToCenter <= EVADE_SCAN_DISTANCE)
	    ordered_enemies.push(it->center);
    }

    if (ordered_enemies.empty()) return false;

    float max_angle = INT_MIN;
    cv::Point2f first_vect = ordered_enemies.top();
    cv::Point2f current_vect = first_vect;

    while (!ordered_enemies.empty()) {
	float current_angle = angle(current_vect, ordered_enemies.top(),
				    frame->getHeight(), frame->getWidth());

	if (max_angle < current_angle) {
	    max_angle = current_angle;
	    _max_first_vec = current_vect;
	    _max_sec_vec = ordered_enemies.top();
	}
	current_vect = ordered_enemies.top();
	ordered_enemies.pop();
    }

    float current_angle =
	angle(current_vect, first_vect, frame->getHeight(), frame->getWidth());
    if (max_angle < current_angle) {
	max_angle = current_angle;
	_max_first_vec = current_vect;
	_max_sec_vec = first_vect;
    }

    _go_to = midVectU(_max_first_vec, _max_sec_vec, frame->getHeight(),
		      frame->getWidth());

    return true;
}

void StrategyEvade::drawPlan(cv::Mat& img)
{
    // cv::line(img, _player_center, _max_first_vec, BIGGER_ENEMIES_COLOR, 1,
    // 8);
    // cv::line(img, _player_center, _max_sec_vec, BIGGER_ENEMIES_COLOR, 1, 8);
    cv::line(img, _player_center, _go_to, BIGGER_ENEMIES_COLOR, 2, 8);
    cv::circle(img, _player_center, (int)EVADE_SCAN_DISTANCE,
	       BIGGER_ENEMIES_COLOR, 1, 8, 0);
}

//==============================================================================
//			  Private algorithmic methods
//==============================================================================

float StrategyEvade::angle(const cv::Point2f& enemy1, const cv::Point2f& enemy2,
			   double height, double width)
{
    cv::Point2f enemy1_center(enemy1.x + width / 2, height / 2 - enemy1.y);
    cv::Point2f enemy2_center(enemy2.x + width / 2, height / 2 - enemy2.y);

    float dot =
	enemy1_center.x * enemy2_center.x + enemy1_center.y * enemy2_center.y;
    float det =
	enemy1_center.x * enemy2_center.y - enemy1_center.y * enemy2_center.x;
    return atan2(det, dot);
}

cv::Point2f StrategyEvade::midVectU(const cv::Point2f& v1,
				    const cv::Point2f& v2, double height,
				    double width)
{
    // screen to cartesian
    cv::Point2f v1_cartesian(v1.x - width / 2, height / 2 - v1.y);
    cv::Point2f v2_cartesian(v2.x - width / 2, height / 2 - v2.y);

    cv::Point2f center = -((v2_cartesian + v1_cartesian) * 0.5);

    center.x = center.x + width / 2;
    center.y = height / 2 - center.y;

    return center;
}
