#ifndef CELL_H
#define CELL_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class Cell
{
public:
    enum class Type {PLAYER, BIGGER_ENEMY, SMALLER_ENEMY, VIRUS, PELLET};

    int id;
    float radius;
    cv::Point2f center;
    float distanceToCenter;
    bool bigger;
    Type type;


    Cell();

    Cell(int id, float radius, cv::Point2f center, float distanceToCenter,
	 bool bigger, Type type);

    inline float distance(const Cell& other);

    inline float distance(const Cell* other);

    static std::string getTypeStr(const Type& type);

    class CellPositionDistanceComparison
    {
	cv::Point2f _player_center;

    public:
	CellPositionDistanceComparison(const cv::Point2f& p)
	{
	    _player_center = p;
	}
	bool operator()(const Cell* cell1, const Cell* cell2) const
	{
	    if (cell1->distanceToCenter > cell2->distanceToCenter)
		return true;
	    else if (cell1->distanceToCenter > cell2->distanceToCenter)
		return false;
	    else {
		if (cell1->center.y > _player_center.y &&
		    cell2->center.y > _player_center.y) {
		    return cell1->center.x > cell2->center.x;

		} else if (cell1->center.y <= _player_center.y &&
			   cell2->center.y <= _player_center.y) {
		    return cell1->center.x < cell2->center.x;

		} else if (cell1->center.y > _player_center.y &&
			   cell2->center.y <= _player_center.y) {
		    return true;

		} else if (cell1->center.y <= _player_center.y &&
			   cell2->center.y > _player_center.y) {
		    return false;
		}
		return false;
	    }
	}
    };
    class CellPositionComparasion
    {
	cv::Point2f _player_center;

    public:
	CellPositionComparasion(const cv::Point2f& p) { _player_center = p; }
	bool operator()(const Cell& cell1, const Cell& cell2) const
	{
	    if (cell1.center.y > _player_center.y &&
		cell2.center.y > _player_center.y) {
		return cell1.center.x > cell2.center.x;

	    } else if (cell1.center.y <= _player_center.y &&
		       cell2.center.y <= _player_center.y) {
		return cell1.center.x < cell2.center.x;

	    } else if (cell1.center.y > _player_center.y &&
		       cell2.center.y <= _player_center.y) {
		return true;

	    } else if (cell1.center.y <= _player_center.y &&
		       cell2.center.y > _player_center.y) {
		return false;
	    }
	    return false;
	}
    };
};

#endif  // CELL_H
