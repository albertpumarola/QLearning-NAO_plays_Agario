#include "frame.h"

//==============================================================================
//				  Constructors
//==============================================================================

Frame::Frame(const cv::Mat& img, const std::vector<cv::Point2f>& centers,
	     const std::vector<float>& radius,
	     const std::vector<std::vector<cv::Point>>& contours_poly,
	     double height, double width, bool is_alive,
	     float previous_player_radious)
    : _height(height),
      _width(width),
      _is_alive(is_alive),
      _previous_player_radious(previous_player_radious)
{
    generateFrameElements(img, centers, radius, contours_poly);
}

//==============================================================================
//				    Getters
//==============================================================================

const std::shared_ptr<std::vector<Cell>> Frame::getBiggerEnemies() const
{
    return std::make_shared<std::vector<Cell>>(_bigger_enemies);
}
const std::shared_ptr<std::vector<Cell>> Frame::getSmallerEnemies() const
{
    return std::make_shared<std::vector<Cell>>(_smaller_enemies);
}
const std::shared_ptr<std::vector<Cell>> Frame::getPellets() const
{
    return std::make_shared<std::vector<Cell>>(_pellets);
}
const std::shared_ptr<std::vector<Cell>> Frame::getViurses() const
{
    return std::make_shared<std::vector<Cell>>(_viruses);
}
const std::shared_ptr<Cell> Frame::getPlayer() const
{
    return std::make_shared<Cell>(_player);
}

cv::Point2f Frame::getLimitMin() const { return _limit_min; }
cv::Point2f Frame::getLimitMax() const { return _limit_max; }

int Frame::getWidth() const { return _width; }
int Frame::getHeight() const { return _height; }

bool Frame::getIsAlive() const { return _is_alive; }
bool Frame::getHasEatenPellet() const
{
    if (_previous_player_radious == 0)
	return false;
    else
	return _previous_player_radious+RADIUS_ADDED_WHEN_EATEN_PELLET< _player.radius;
}
bool Frame::getHasEatenCell() const { return false; }

//==============================================================================
//			   Public algorithmic methods
//==============================================================================

void Frame::display(cv::Mat& img_display)
{
    // display enemies
    for (size_t i = 0; i < _bigger_enemies.size(); i++) {
	cv::circle(img_display, _bigger_enemies[i].center,
		   (int)_bigger_enemies[i].radius, BIGGER_ENEMIES_COLOR, 2, 8,
		   0);
	cv::putText(img_display, std::to_string(_bigger_enemies[i].id),
		    _bigger_enemies[i].center, 1, 0.8, BIGGER_ENEMIES_COLOR, 1,
		    CV_AA);
    }

    for (size_t i = 0; i < _smaller_enemies.size(); i++) {
	cv::circle(img_display, _smaller_enemies[i].center,
		   (int)_smaller_enemies[i].radius, SMALLER_ENEMIES_COLOR, 2, 8,
		   0);
	cv::putText(img_display, std::to_string(_smaller_enemies[i].id),
		    _smaller_enemies[i].center, 1, 0.8, SMALLER_ENEMIES_COLOR,
		    1, CV_AA);
    }

    // display pellets
    for (size_t i = 0; i < _pellets.size(); i++) {
	cv::circle(img_display, _pellets[i].center, (int)_pellets[i].radius,
		   PELLET_COLOR, 2, 8, 0);
	cv::putText(img_display, std::to_string(_pellets[i].id),
		    _pellets[i].center, 1, 0.8, PELLET_COLOR, 1, CV_AA);
    }

    // display viruses
    for (size_t i = 0; i < _viruses.size(); i++) {
	cv::circle(img_display, _viruses[i].center, (int)_viruses[i].radius,
		   VIRUS_COLOR, 2, 8, 0);

	cv::putText(img_display, std::to_string(_viruses[i].id),
		    _viruses[i].center, 1, 0.8, VIRUS_COLOR, 1, CV_AA);
    }

    // display player
    cv::circle(img_display, _player.center, (int)_player.radius, PLAYER_COLOR,
	       2, 8, 0);
    cv::putText(img_display, std::to_string(_player.id), _player.center, 1, 0.8,
		PLAYER_COLOR, 1, CV_AA);

    // display borders
    cv::line(img_display, cv::Point2f(_limit_min.x, _height),
	     cv::Point2f(_limit_min.x, 0), LIMIT_COLOR, 2, 8);
    cv::line(img_display, cv::Point2f(0, _limit_min.y),
	     cv::Point2f(_width, _limit_min.y), LIMIT_COLOR, 2, 8);
    cv::line(img_display, cv::Point2f(_limit_max.x, _height),
	     cv::Point2f(_limit_max.x, 0), LIMIT_COLOR, 2, 8);
    cv::line(img_display, cv::Point2f(0, _limit_max.y),
	     cv::Point2f(_width, _limit_max.y), LIMIT_COLOR, 2, 8);
}

//==============================================================================
//			  Private algorithmic methods
//==============================================================================

void Frame::generateFrameElements(
    const cv::Mat& img, const std::vector<cv::Point2f>& centers,
    const std::vector<float>& radius,
    const std::vector<std::vector<cv::Point>>& contours_poly)
{
    if (radius.size() == 0) return;

    // get min radius
    float min_radius = FLT_MAX;
    for (size_t i = 0; i < radius.size(); ++i) {
	if (min_radius > radius[i]) min_radius = radius[i];
    }

    // get distances
    std::vector<float> distances(centers.size());
    float min_dist = INT_MAX;
    float min_pos = -1;
    for (size_t i = 0; i < centers.size(); ++i) {
	distances[i] = distToCenter(centers[i]);
	if (radius[i] <= CELL_MAX_RADIUS && radius[i] > CELL_MIN_RADIUS) {
	    if (distances[i] < min_dist) {
		min_dist = distances[i];
		min_pos = i;
	    }
	}
    }

    // get player
    _player = Cell(min_pos, radius[min_pos], centers[min_pos],
		   distances[min_pos], false, Cell::Type::PLAYER);

    // get enemies & pellets
    for (size_t i = 0; i < centers.size(); ++i) {
	if (i != min_pos && radius[i] <= CELL_MAX_RADIUS) {
	    if ((radius[i] <= min_radius + PELLET_RADIUS_RANGE &&
		 radius[i] >= min_radius - PELLET_RADIUS_RANGE))
		_pellets.emplace_back(i, radius[i], centers[i], distances[i],
				      false, Cell::Type::PELLET);
	    else {  // enemies or viruses

		cv::Vec3b color = img.at<cv::Vec3b>(centers[i].y, centers[i].x);
		if (color.val[0] == 49 && color.val[1] == 255 &&
		    color.val[2] == 49)  // virus
		    _viruses.emplace_back(i, radius[i], centers[i],
					  distances[i], true,
					  Cell::Type::VIRUS);
		else if (radius[i] < _player.radius)  // smaller enemy
		    _smaller_enemies.emplace_back(i, radius[i], centers[i],
						  distances[i], true,
						  Cell::Type::SMALLER_ENEMY);
		else  // bigger enemy
		    _bigger_enemies.emplace_back(i, radius[i], centers[i],
						 distances[i], false,
						 Cell::Type::BIGGER_ENEMY);
	    }
	}
    }

    // get borders
    _limit_max = cv::Point2f(INT_MIN, INT_MIN);
    _limit_min = cv::Point2f(INT_MAX, INT_MAX);
    for (size_t i = 0; i < centers.size(); ++i) {
	if (radius[i] <= CELL_MAX_RADIUS && radius[i] > CELL_MIN_RADIUS) {
	    if (centers[i].x < _limit_min.x) _limit_min.x = centers[i].x;
	    if (centers[i].x > _limit_max.x) _limit_max.x = centers[i].x;

	    if (centers[i].y < _limit_min.y) _limit_min.y = centers[i].y;
	    if (centers[i].y > _limit_max.y) _limit_max.y = centers[i].y;
	}
    }
}

float Frame::distToCenter(const cv::Point2f& point)
{
    return std::sqrt(std::pow(point.x - _width / 2, 2) +
		     std::pow(point.y - _height / 2, 2));
}
