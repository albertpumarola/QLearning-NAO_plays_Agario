#ifndef QLEARNING_H
#define QLEARNING_H

#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include "globals.h"
#include "csv.h"

class QLearning
{
public:
    QLearning(int num_states, long num_actions, double gamma,
	      double random_exploration_factor);

    int getNextAction(long current_state,
		      const std::vector<bool>& feasible_actions);
    std::vector<size_t> getOrderedActions(long current_state);
    int learn(long current_state, const std::vector<bool>& feasible_actions,
	      bool is_alive, bool has_eaten_pellet = false,
	      bool has_eaten_cell = false);
    void saveQToFile(const std::string& file_path);
    void loadQ();

private:
    int _num_states;
    int _num_actions;
    double _gamma;
    int _current_iteraton;
    int _prev_state;
    int _prev_action;
    std::vector<std::vector<int>> _Q;
    std::vector<std::vector<int>> _R;
    double _random_exploration_factor;
    CSV<int> _csv;

    std::uniform_int_distribution<int> _int_uni;
    std::uniform_real_distribution<double> _real_uni;
    std::mt19937 _rng;

    void initRMat();
    inline int randAction();
    inline int randConstrainedAction(const std::vector<bool>& feasible_actions);
    int query(int action, int inst_reward, int current_state, int prev_state,
	      const std::vector<bool>& feasible_actions);
    void queryDead();
    void updateQ(int new_action, int prev_action, int inst_reward,
		 int current_state, int prev_state);
    int generateAction(int current_state,
		       const std::vector<bool>& feasible_actions,
		       bool has_exploration);
    inline int maxConstrainedElement(const std::vector<int>& current_state,
			      const std::vector<bool>& feasible_actions);
    void readQFromFile();
};

#endif  // QLEARNING_H
