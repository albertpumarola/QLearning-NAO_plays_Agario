#ifndef STRATEGYEVADE_H
#define STRATEGYEVADE_H

#include "strategy.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <queue>

class StrategyEvade : public Strategy
{
public:
    bool plan(const Frame* frame);
    void drawPlan(cv::Mat& img);

private:
    cv::Point2f _player_center;
    cv::Point2f _max_first_vec, _max_sec_vec;

    inline float angle(const cv::Point2f& enemy1, const cv::Point2f& enemy2,
		       double height, double width);
    inline cv::Point2f midVectU(const cv::Point2f& vec1,
				const cv::Point2f& vec2, double height,
				double width);

    class enemies_comparasion
    {
	cv::Point2f _player_center;

    public:
	enemies_comparasion(const cv::Point2f& p) { _player_center = p; }
	bool operator()(const cv::Point2f& enemy1,
			const cv::Point2f& enemy2) const
	{
	    if (enemy1.y > _player_center.y && enemy2.y > _player_center.y) {
		return enemy1.x > enemy2.x;

	    } else if (enemy1.y <= _player_center.y &&
		       enemy2.y <= _player_center.y) {
		return enemy1.x < enemy2.x;

	    } else if (enemy1.y > _player_center.y &&
		       enemy2.y <= _player_center.y) {
		return true;

	    } else if (enemy1.y <= _player_center.y &&
		       enemy2.y > _player_center.y) {
		return false;
	    }
	    return false;
	}
    };
};

#endif  // STRATEGYEVADE_H
