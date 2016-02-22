#include "strategy.h"

//==============================================================================
//				    Getters
//==============================================================================

cv::Point2f Strategy::getGoTo() const { return _go_to; }

//==============================================================================
//			   Public algorithmic methods
//==============================================================================

bool Strategy::plan(const Frame* frame) {}
bool Strategy::plan(const Frame* frame, int option) {}

