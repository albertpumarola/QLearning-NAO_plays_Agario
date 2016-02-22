#include "qlearning.h"

//==============================================================================
//				  Constructors
//==============================================================================

QLearning::QLearning(int num_states, long num_actions, double gamma,
		     double random_exploration_factor)
    : _num_states(num_states),
      _num_actions(num_actions),
      _gamma(gamma),
      _current_iteraton(0),
      _prev_state(-1),
      _prev_action(-1),
      _Q(num_states, std::vector<int>(num_actions, 0)),
      _R(num_states, std::vector<int>(num_actions, 0)),
      _random_exploration_factor(random_exploration_factor),
      _int_uni(0, num_actions - 1),
      _real_uni(0, 1)
{
    initRMat();
}

//==============================================================================
//				    Getters
//==============================================================================
int QLearning::getNextAction(long current_state,
			     const std::vector<bool>& feasible_actions)
{
    return generateAction(current_state, feasible_actions, false);
}

std::vector<size_t> QLearning::getOrderedActions(long current_state)
{
    std::vector<size_t> idx(_Q[current_state].size());
    for (size_t i = 0; i != idx.size(); ++i) idx[i] = i;

    std::sort(idx.begin(), idx.end(), [&](size_t i1, size_t i2) {
	return _Q[current_state][i1] > _Q[current_state][i2];
    });

    return idx;
}

//==============================================================================
//			   Public algorithmic methods
//==============================================================================

int QLearning::learn(long current_state,
		     const std::vector<bool>& feasible_actions, bool is_alive,
		     bool has_eaten_pellet, bool has_eaten_cell)
{
    if (!is_alive) {
	queryDead();
	return -1;
    }

    int inst_reward = R_HAS_SURVIVED;
    inst_reward += has_eaten_pellet ? R_HAS_EATEN_PELLET : 0;
    inst_reward += has_eaten_cell ? R_HAS_EATEN_CELL : 0;

    int new_action;
    if (_prev_action == -1 || _prev_state == -1)
	new_action = randAction();
    else
	new_action = query(_prev_action, inst_reward, current_state,
			   _prev_state, feasible_actions);

    _prev_state = current_state;
    _prev_action = new_action;

    return new_action;
}

void QLearning::saveQToFile(const std::string& file_path)
{
    _csv.write(_Q, file_path);
}

void QLearning::loadQ() { readQFromFile(); }

//==============================================================================
//			  Private algorithmic methods
//==============================================================================

void QLearning::initRMat()
{
    auto dangerous_state = [](int state) {
	int num_dangerous = 0;
	int num_analized_rays = 0;
	while (num_analized_rays <= NUM_RAYS) {
	    if (state % 2) num_dangerous++;
	    state /= 2;
	    num_analized_rays++;
	    if (num_dangerous > 2) return true;
	}
	return false;
    };

    auto max_pellets_region =
	[](int state) -> int { return state / (std::pow(2, NUM_RAYS)); };

    auto in_best_region = [](int state) -> bool {
	return int(state / (std::pow(2, NUM_RAYS))) == 4;
    };

    for (int state = 0; state < _num_states; ++state) {
	if (dangerous_state(state)) _R[state][EVADE] = 4;
	//if (in_best_region(state)) _R[state][EAT_PELLET] = 2;
	//_R[state][EAT_PELLET] = 3;
	//_R[state][GO_REGION + max_pellets_region(state)] = 2;
    }
}

int QLearning::randAction() { return _int_uni(_rng); }

int QLearning::randConstrainedAction(const std::vector<bool>& feasible_actions)
{
    int action;
    do {
	action = randAction();
    } while (!feasible_actions[action]);
    return action;
}

int QLearning::query(int prev_action, int inst_reward, int current_state,
		     int prev_state, const std::vector<bool>& feasible_actions)
{
    auto new_action = generateAction(current_state, feasible_actions, true);
    _Q[prev_state][prev_action] = inst_reward + _R[prev_state][prev_action] +
				  _gamma * _Q[current_state][new_action];

    return new_action;
}

void QLearning::queryDead()
{
    if (_prev_state != -1 && _prev_action != -1)
	_Q[_prev_state][_prev_action] = R_HAS_DIED;
    _prev_state = -1;
    _prev_action = -1;
}

int QLearning::generateAction(int current_state,
			      const std::vector<bool>& feasible_actions,
			      bool has_exploration)
{
    auto random_exploration = _real_uni(_rng) < _random_exploration_factor;
    if (has_exploration && random_exploration)
	return randConstrainedAction(feasible_actions);
    else
	return maxConstrainedElement(_Q[current_state], feasible_actions);
}

int QLearning::maxConstrainedElement(const std::vector<int>& v,
				     const std::vector<bool>& feasible_actions)
{
    int max_element = v[0];
    int max_element_i = 0;
    for (size_t i = 1; i < v.size(); ++i) {
	if (max_element < v[i] && feasible_actions[i]) {
	    max_element = v[i];
	    max_element_i = i;
	}
    }

    return max_element_i;
}

void QLearning::readQFromFile() { _csv.read(_Q, Q_FILE); }
