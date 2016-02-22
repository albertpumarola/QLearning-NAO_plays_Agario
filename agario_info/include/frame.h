#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include "cell.h"
#include "globals.h"
#include <climits>
#include <float.h>
#include <math.h>
#include <iostream>
#include <memory>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class Frame
{
public:
    Frame(const cv::Mat& img, const std::vector<cv::Point2f>& centers,
	  const std::vector<float>& radius,
	  const std::vector<std::vector<cv::Point>>& contours_poly,
	  double height, double width, bool _is_alive, float previous_player_radious);

    const std::shared_ptr<std::vector<Cell>> getBiggerEnemies() const;
    const std::shared_ptr<std::vector<Cell>> getSmallerEnemies() const;
    const std::shared_ptr<std::vector<Cell>> getPellets() const;
    const std::shared_ptr<std::vector<Cell>> getViurses() const;
    const std::shared_ptr<Cell> getPlayer() const;
    cv::Point2f getLimitMin() const;
    cv::Point2f getLimitMax() const;
    int getWidth() const;
    int getHeight() const;
    bool getIsAlive() const;
    bool getHasEatenPellet() const;
    bool getHasEatenCell() const;

    void display(cv::Mat& img_display);

private:
    std::vector<Cell> _bigger_enemies;
    std::vector<Cell> _smaller_enemies;
    std::vector<Cell> _pellets;
    std::vector<Cell> _viruses;
    Cell _player;
    int _height;
    int _width;
    cv::Point2f _limit_min;
    cv::Point2f _limit_max;

    bool _is_alive;
    bool _has_eaten_pellet;
    bool _has_eaten_cell;

    float _previous_player_radious;

    float distToCenter(const cv::Point2f& point);
    void generateFrameElements(
	const cv::Mat& img, const std::vector<cv::Point2f>& centers,
	const std::vector<float>& radius,
	const std::vector<std::vector<cv::Point>>& contours_poly);
};

#endif  // FRAME_H
