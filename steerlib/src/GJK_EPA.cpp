#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
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
bool isConvex(const std::vector<Util::Vector>& _shape) {
		if (_shape.size() <= 2) {
	std::cerr << "it`s not a shape" << std::endl;
	}
	for (int i = 0; i < _shape.size(); ++i) {
	int next = i + 1;
	if (next == _shape.size()) {
	next = 0;
	}
	int before = i - 1;
	if (before == -1) {
	before = _shape.size() - 1;
	}


	Util::Vector v1 = _shape[i] - _shape[before];
	Util::Vector v2 = _shape[next] - _shape[i];
	
	float mul = v1.z*v2.x - v1.x*v2.z;

	if (mul < 0) {
	//	std::cout << "not a convex" << std::endl;
		//std::cout << _shape[before].x<<","<< _shape[before].y<<","<< _shape[before].z << std::endl;
		//std::cout << _shape[i].x << "," << _shape[i].y << "," << _shape[i].z << std::endl;
		//std::cout << _shape[next].x << "," << _shape[next].y << "," << _shape[next].z << std::endl;

		return false;
	}
	//std::cout << "its convex" << std::endl;

	}
	return true;

}


Util::Vector getCenter(const std::vector<Util::Vector>& _shape)
{
	Util::Vector c;
	c.x = 0;
	c.y = 0;
	c.z = 0;
	for (std::vector<Util::Vector>::const_iterator it = _shape.begin(); it != _shape.end(); ++it)
	{
		c.x += it->x;
		c.y += it->y;
		c.z += it->z;
	}
	if (!_shape.empty())
	{
		c.x /= _shape.size();
		c.y /= _shape.size();
		c.z /= _shape.size();
	}
	return c;
}

void negate(Util::Vector &direction) {
	direction.x = -direction.x;
	direction.y = -direction.y;
	direction.z = -direction.z;
}


bool containsOrigin(std::vector<Util::Vector> simplex, Util::Vector *d) {
	return false;
}

bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	Util::Vector zero(0, 0, 0);
	Util::Vector start_a(0, 0, 0), start_b(0, 0, 0);
	std::vector<Util::Vector> simplex;
	Util::Vector direction;
	
	direction = getCenter(_shapeB) - getCenter(_shapeA);

	simplex.push_back(support(_shapeA, _shapeB, direction));

	negate(direction);

	while (true) {
		simplex.push_back(support(_shapeA, _shapeB, direction));
		if (simplex[simplex.size() - 1].x*direction.x+ simplex[simplex.size() - 1].y*direction.y+ simplex[simplex.size() - 1].z*direction.z<=0) {
			return false;
		}
		else {
			if (containsOrigin(simplex,&direction)) {
				return true;
			}
		}
	
	}


	return false; // There is no collision
}

