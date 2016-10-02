#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

Util::Vector get_far_point(const std::vector<Util::Vector>& _shape, Util::Vector direction) {
	//the max dot produt is the fathest point.
	float max = INT_MIN;
	Util::Vector result;
	for (auto it = _shape.begin(); it != _shape.end(); it++) {
		float tmp = (*it) * direction;
		if (tmp > max) {
			max = tmp;
			result = (*it);
		}
	}
	return result;
}
Util::Vector support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector direction) {
	Util::Vector point1 = get_far_point(_shapeA, direction);
	Util::Vector point2 = get_far_point(_shapeB, direction * (-1));
	return point1 - point2;
}
//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	Util::Vector zero(0,0,0);
	Util::Vector start_a(0,0,0), start_b(0,0,0);
	std::vector<Util::Vector> simplex;
	Util::Vector direction;
	
	simplex.push_back(support(_shapeA, _shapeB, direction));
	return false; // There is no collision
}