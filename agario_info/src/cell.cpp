#include "cell.h"

//==============================================================================
//				  Constructors
//==============================================================================
Cell::Cell() {}

Cell::Cell(int id, float radius, cv::Point2f center, float distanceToCenter,
	   bool bigger, Type type)
    : id(id),
      radius(radius),
      center(center),
      distanceToCenter(distanceToCenter),
      bigger(bigger),
      type(type)
{
}

//==============================================================================
//				    Getters
//==============================================================================

std::string Cell::getTypeStr(const Type& type)
{
    switch (type) {
	case Type::PLAYER:
	    return std::string("PLAYER");
	case Type::BIGGER_ENEMY:
	    return std::string("BIGGER_ENEMY");
	case Type::SMALLER_ENEMY:
	    return std::string("SMALLER_ENEMY");
	case Type::VIRUS:
	    return std::string("VIRUS");
	case Type::PELLET:
	    return std::string("PELLET");
	default:
	    return std::string("None");
    }
}

//==============================================================================
//			 Public algorithmic functions
//==============================================================================
float Cell::distance(const Cell& other)
{
    return std::sqrt(std::pow(center.x - other.center.x, 2) +
		     std::pow(center.y - other.center.y / 2, 2));
}

float Cell::distance(const Cell* other)
{
    return std::sqrt(std::pow(center.x - other->center.x, 2) +
		     std::pow(center.y - other->center.y / 2, 2));
}
