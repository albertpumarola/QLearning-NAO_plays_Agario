#include "decisionMaker.h"

//==============================================================================
//				  Constructors
//==============================================================================

DecisionMaker::DecisionMaker(bool train_mode)
    : _qlearning(Q_NUM_STATES, NUM_ACTIONS, Q_GAMMA, Q_EXPLORATION_FACTOR),
      _s_go_region(NUM_REGIONS_X * NUM_REGIONS_Y),
      _feasible_actions(3 + NUM_REGIONS_X * NUM_REGIONS_Y),
      _ray_dangerous(NUM_RAYS),
      _action_todo(-1),
      _num_game(0),
      _train_mode(train_mode)
{
    if (!train_mode) _qlearning.loadQ();
}

//==============================================================================
//				    Getters
//==============================================================================

int DecisionMaker::getCurrentGame() const { return _num_game; }

cv::Point2f DecisionMaker::getGoal() const
{

    auto player_center =  _frame->getPlayer()->center;
    if (_action_todo == -1)
	return player_center;
    else if (_action_todo == 0)
	return 2*_s_eat_pellets.getGoTo();
    else if (_action_todo == 1)
	return _s_chase_senemy.getGoTo();
    else if (_action_todo == 2)
	return _s_evade.getGoTo();
    else
	return _s_go_region[_action_todo - 3].getGoTo();
}

//==============================================================================
//			   Public algorithmic methods
//==============================================================================

void DecisionMaker::execute(const Frame* frame)
{
    if (_train_mode)
	executeLearning(frame);
    else
	executeModel(frame);
}

void DecisionMaker::drawPelletsRegions(cv::Mat& img)
{
    int row = _best_pellets_region / NUM_PELLETS_REGIONS_X;
    int col = _best_pellets_region - row * NUM_PELLETS_REGIONS_X;
    double width_block = _frame->getWidth() / NUM_PELLETS_REGIONS_X;
    double height_block = _frame->getHeight() / NUM_PELLETS_REGIONS_Y;
    cv::Point2f p1(col * width_block, row * height_block);
    cv::Point2f p2(col * width_block + width_block,
		   row * height_block + height_block);
    cv::Mat roi = img(cv::Rect(p1, p2));
    cv::Mat color(roi.size(), CV_8UC3, PELLET_COLOR);
    double alpha = 0.3;
    cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
}

void DecisionMaker::drawRays(cv::Mat& img)
{
    float angle_step = 2 * M_PI / NUM_RAYS;
    cv::Point2f player_center = _frame->getPlayer()->center;
    for (int i = 0; i < NUM_RAYS; ++i) {
	double angle = i * angle_step;
	float ray_end_x = RAYS_RANGE * cos(angle) + player_center.x;
	float ray_end_y = RAYS_RANGE * sin(angle) + player_center.y;
	cv::Point2f end_ray(ray_end_x, ray_end_y);

	if (_ray_dangerous[i]) {
	    cv::line(img, _frame->getPlayer()->center, end_ray, DANGEROUS_COLOR,
		     2, 8);
	} else {
	    cv::line(img, _frame->getPlayer()->center, end_ray, SAFE_COLOR, 2,
		     8);
	}
    }
}

void DecisionMaker::drawPlan(cv::Mat& img)
{
    if (_action_todo == -1)
	return;
    else if (_action_todo == 0)
	_s_eat_pellets.drawPlan(img);
    else if (_action_todo == 1)
	_s_chase_senemy.drawPlan(img);
    else if (_action_todo == 2)
	_s_evade.drawPlan(img);
    else
	_s_go_region[_action_todo - 3].drawPlan(img);
}

void DecisionMaker::drawAllPlans(cv::Mat& img)
{
    if (_feasible_actions[0]) _s_eat_pellets.drawPlan(img);

    if (_feasible_actions[1]) _s_chase_senemy.drawPlan(img);

    if (_feasible_actions[2]) _s_evade.drawPlan(img);

    for (int region = 0; region < NUM_REGIONS_X * NUM_REGIONS_Y; ++region) {
	if (_feasible_actions[3 + region]) _s_go_region[region].drawPlan(img);
    }
}

void DecisionMaker::saveQLearningModel(const std::string& path)
{
    _qlearning.saveQToFile(path);
}

//==============================================================================
//			  Private algorithmic methods
//==============================================================================
void DecisionMaker::executeLearning(const Frame* frame)
{
    _frame = frame;
    planAllStrategies();
    auto current_state = getCurrentState();
    auto is_alive = _frame->getIsAlive();
    auto has_eaten_pellet = _frame->getHasEatenPellet();
    auto has_eaten_cell = _frame->getHasEatenCell();

    if (!is_alive) hasDied();

    _action_todo = _qlearning.learn(current_state, _feasible_actions, is_alive,
				    has_eaten_pellet, has_eaten_cell);
}

void DecisionMaker::executeModel(const Frame* frame)
{
    _frame = frame;
    auto current_state = getCurrentState();
    auto ordered_actions = _qlearning.getOrderedActions(current_state);

    bool feasible = false;
    size_t i_action = 0;
    while (!feasible && i_action < ordered_actions.size()) {
	if (ordered_actions[i_action] == 0)
	    feasible = _s_eat_pellets.plan(frame);

	if (ordered_actions[i_action] == 1)
	    feasible = _s_chase_senemy.plan(frame);

	if (ordered_actions[i_action] == 2)
	    feasible = _s_evade.plan(frame);

	if (ordered_actions[i_action] > 2)
	    feasible = _s_go_region[ordered_actions[i_action]-3].plan(frame, ordered_actions[i_action]-3);

	i_action++;
    }
    if(feasible) _action_todo = ordered_actions[i_action-1];
    else _action_todo = 0;
}

void DecisionMaker::hasDied()
{
    _num_game++;
    if (_num_game % SAVE_Q_EVERY == 0)
	_qlearning.saveQToFile(Q_SAVE_PATH + std::to_string(_num_game) +
			       ".csv");
    std::cout << "NUM GAME: " << _num_game << std::endl;
}

void DecisionMaker::generatePelletsGrid()
{
    auto pellets = _frame->getPellets();
    double width_block = _frame->getWidth() / NUM_PELLETS_REGIONS_X;
    double height_block = _frame->getHeight() / NUM_PELLETS_REGIONS_Y;
    std::vector<int> pellets_grid(NUM_PELLETS_REGIONS_X *
				  NUM_PELLETS_REGIONS_Y);

    for (const auto& pellet : *pellets) {
	int block_x = pellet.center.x / width_block;
	block_x = (block_x == NUM_REGIONS_X) ? NUM_REGIONS_X - 1 : block_x;
	int block_y = pellet.center.y / height_block;
	block_y = (block_y == NUM_REGIONS_Y) ? NUM_REGIONS_Y - 1 : block_y;
	pellets_grid[block_y * NUM_PELLETS_REGIONS_X + block_x]++;
    }

    pellets_grid.erase(pellets_grid.begin()+6);
    _best_pellets_region =
	std::max_element(pellets_grid.begin(), pellets_grid.end()) -
	pellets_grid.begin();
}

void DecisionMaker::planAllStrategies()
{
    std::vector<std::thread> threads;

    auto execute_strategy = [&](Strategy& s, int i_plan, const Frame* frame) {
	_feasible_actions[i_plan] = s.plan(frame);
    };
    threads.push_back(std::thread(execute_strategy, std::ref(_s_eat_pellets),
				  EAT_PELLET, _frame));
    threads.push_back(std::thread(execute_strategy, std::ref(_s_chase_senemy),
				  CHASE_ENEMY, _frame));
    threads.push_back(
	std::thread(execute_strategy, std::ref(_s_evade), EVADE, _frame));

    auto execute_strategy_option =
	[&](Strategy& s, int i_plan, const Frame* frame, int option) {
	    _feasible_actions[i_plan] = s.plan(frame, option);
	};
    for (int region = 0; region < NUM_REGIONS_X * NUM_REGIONS_Y; ++region) {
	threads.push_back(std::thread(execute_strategy_option,
				      std::ref(_s_go_region[region]),
				      GO_REGION + region, _frame, region));
    }

    for (auto& th : threads) th.join();
}

void DecisionMaker::intersectRays()
{
    std::vector<Cell*> cells_collision;
    auto bigger_enemies = _frame->getBiggerEnemies();
    cells_collision.reserve(bigger_enemies->size());
    for (auto& cell : *bigger_enemies) cells_collision.push_back(&cell);
    auto viruses = _frame->getViurses();
    cells_collision.reserve(viruses->size());
    for (auto& cell : *viruses) cells_collision.push_back(&cell);

    auto player_cell = _frame->getPlayer();

    std::vector<std::thread> threads(NUM_RAYS);
    float angle_step = 2 * M_PI / NUM_RAYS;
    for (int i = 0; i < NUM_RAYS; ++i) {
	double angle = i * angle_step;
	threads[i] =
	    std::thread(&DecisionMaker::rayIntersection, this, angle,
			std::ref(player_cell), std::ref(cells_collision), i);
    }
    for (auto& thread : threads) thread.join();
}

void DecisionMaker::rayIntersection(double ray_angle,
				    const std::shared_ptr<Cell> player_cell,
				    const std::vector<Cell*>& cells,
				    int i_intersect)
{
    _ray_dangerous[i_intersect] = false;
    for (auto& cell : cells) {
	float ray_end_x = RAYS_RANGE * cos(ray_angle) + player_cell->center.x;
	float ray_end_y = RAYS_RANGE * sin(ray_angle) + player_cell->center.y;
	cv::Point2f end_ray(ray_end_x, ray_end_y);
	bool intersect = rayCircleIntersect(player_cell->center, end_ray,
					    cell->center, cell->radius);
	if (intersect) _ray_dangerous[i_intersect] = true;
    }
}

//TODO Improve detection
bool DecisionMaker::rayCircleIntersect(const cv::Point2f& begin_ray,
				       const cv::Point2f& end_ray,
				       const cv::Point2f& circle_center,
				       float circle_r)
{
    cv::Point2f d = end_ray - begin_ray;
    cv::Point2f f = begin_ray - circle_center;
    float a = d.dot(d);
    float b = 2 * f.dot(d);
    float c = f.dot(f) - std::pow(circle_r, 2);

    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
	// no intersection
	return false;
    } else {
	// ray didn't totally miss sphere,
	// so there is a solution to
	// the equation.

	discriminant = sqrt(discriminant);

	// either solution may be on or off the ray so need to test both
	// t1 is always the smaller value, because BOTH discriminant and
	// a are nonnegative.
	float t1 = (-b - discriminant) / (2 * a);
	float t2 = (-b + discriminant) / (2 * a);

	// 3x HIT cases:
	//          -o->             --|-->  |            |  --|->
	// Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

	// 3x MISS cases:
	//       ->  o                     o ->              | -> |
	// FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

	if (t1 >= 0 && t1 <= 1) {
	    // t1 is the intersection, and it's closer than t2
	    // (since t1 uses -b - discriminant)
	    // Impale, Poke
	    return true;
	}

	// here t1 didn't intersect so we are either started
	// inside the sphere or completely past it
	if (t2 >= 0 && t2 <= 1) {
	    // ExitWound
	    return true;
	}

	// no intn: FallShort, Past, CompletelyInside
	return false;
    }
}

long DecisionMaker::getCurrentState()
{
    generatePelletsGrid();
    intersectRays();

    long state = 0;
    for (size_t i = 0; i < _ray_dangerous.size(); i++) {
	state += std::pow(2, i) * (long)_ray_dangerous[i];
    }

    state += std::pow(2, _ray_dangerous.size()) * (long)_best_pellets_region;

    return state;
}
