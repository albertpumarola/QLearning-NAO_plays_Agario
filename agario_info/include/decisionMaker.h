#ifndef DECISIONMAKER_H
#define DECISIONMAKER_H

#include "frame.h"
#include "globals.h"
#include "strategyEatPellets.h"
#include "strategyChaseSmallerEnemy.h"
#include "strategyEvade.h"
#include "strategyGoRegion.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <algorithm>
#include <math.h>
#include <queue>
#include <vector>
#include <thread>
#include <limits.h>
#include "qlearning.h"

class DecisionMaker
{
public:
    DecisionMaker(bool train_mode);

    cv::Point2f getGoal() const;
    int getCurrentGame() const;
    void execute(const Frame* frame);
    void drawPlan(cv::Mat& img);
    void drawPelletsRegions(cv::Mat& img);
    void drawRays(cv::Mat& img);
    void drawAllPlans(cv::Mat& img);
    void saveQLearningModel(const std::string& path);

private:
    const Frame* _frame;
    QLearning _qlearning;

    StrategyEatPellets _s_eat_pellets;
    StrategyChaseSmallerEnemy _s_chase_senemy;
    StrategyEvade _s_evade;
    std::vector<StrategyGoRegion> _s_go_region;

    std::vector<bool> _feasible_actions;

    int _best_pellets_region;
    std::vector<bool> _ray_dangerous;

    int _action_todo;
    int _num_game;
    bool _train_mode;

    void executeLearning(const Frame* frame);
    void executeModel(const Frame* frame);
    void hasDied();
    void generatePelletsGrid();
    void planAllStrategies();
    void intersectRays();
    void rayIntersection(double ray_angle,
			 const std::shared_ptr<Cell> player_cell,
			 const std::vector<Cell*>& cells, int i_intersect);
    inline bool rayCircleIntersect(const cv::Point2f& begin_ray,
			    const cv::Point2f& end_ray,
			    const cv::Point2f& circle_center, float circle_r);
    inline long getCurrentState();
};

// Reward Matrix
// states: eat, chase, evade;
// action: eat, chase, evade;

#endif  // DECISIONMAKER_H
