#include "obstacles/GJK_EPA.h"
#define TOLERANCE 0.000001

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
//	std::cout << "it`s not a shape" << std::endl;
	}
		int flag = 0;
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
	if (i == 0) flag = flag + mul;
	if (mul *flag< 0) {
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


bool containsOrigin(std::vector<Util::Vector> &simplex, Util::Vector &d) {
	
	
	Util::Vector A = simplex[simplex.size()-1]; // the last added simplex
	Util::Vector AO = A * (-1);
	if (simplex.size() == 3)
	{


		Util::Vector B = simplex[simplex.size() - 2]; // the second last added simplex
		Util::Vector C = simplex[simplex.size() - 3];
		

		Util::Vector AB = B - A;
		Util::Vector AC = C - A;
		
		Util::Vector prep_AB =  AB*(AB*AC)-AC*(AB*AB) ; // (AB X AC) X AB;  
		Util::Vector prep_AC = AC*(AC*AB)-AB*(AC*AC) ; // (AC X AB) X AC;  

		if (prep_AB*AO > 0) {
			simplex.erase(simplex.begin()+ simplex.size() - 3);
			d = prep_AB;
		}
		else {
			if (prep_AC*AO > 0) {
				simplex.erase(simplex.begin() + simplex.size() - 2);
				d = prep_AC;
			}
			else {
//				std::cout << "here" << std::endl;
				return true;

			}
		}
		
	}

	else {
	Util::Vector B = simplex[simplex.size() - 2]; // the second last added simplex
	Util::Vector AB = B - A;
	
	Util::Vector prep_AB = AO*(AB*AB) - AB*(AB*AO);
	d = prep_AB;
	}
	//std::cout << "simplex's size less than 3" << std::endl;

	
	return false;
}
SteerLib::Edge find_closest_edge(std::vector<Util::Vector>& simple) {
	SteerLib::Edge closet_e;
	closet_e.distance = DBL_MAX;
	for (int i = 0; i < simple.size(); i++) {
		int j = (i + 1 == simple.size()) ? 0 : (i + 1);
		Util::Vector a = simple[i];
		Util::Vector b = simple[j];
		Util::Vector ab = b - a;
		Util::Vector ao = (-1) *a;
		//Util::Vector ao = (-1) *simple[i]; 
		Util::Vector prep_AB = ao*(ab*ab) - ab*(ab*ao);
		prep_AB = Util::normalize(prep_AB);
		double d = fabs(ao * prep_AB);
		if (d < closet_e.distance) {
			closet_e.distance = d;
			closet_e.normal = (-1) * prep_AB;
			closet_e.index = j;
		}
	}
	return closet_e;
}
bool EPA(std::vector<Util::Vector>& simplex, float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
	while (true) {
		SteerLib::Edge closest_e = find_closest_edge(simplex);
		Util::Vector p = support(_shapeA, _shapeB, closest_e.normal);
		double distance =  p*closest_e.normal;
		if (fabs(distance - closest_e.distance) < TOLERANCE) {
			return_penetration_vector = closest_e.normal;
			return_penetration_depth = distance;
			return true;
		}
		else {
			simplex.insert(simplex.begin() + closest_e.index, p);
		}
	}
}
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{

	if (!isConvex(_shapeA)) {
		std::cout << "A not convex" << std::endl;
	}
	if (!isConvex(_shapeB)) {
		std::cout << "B not convex" << std::endl;
	}

	Util::Vector zero(0, 0, 0);
	Util::Vector start_a(0, 0, 0), start_b(0, 0, 0);
	std::vector<Util::Vector> simplex;
	Util::Vector direction;
	direction = getCenter(_shapeB) - getCenter(_shapeA);
//	std::cout << "d:" << direction << std::endl;

	simplex.push_back(support(_shapeA, _shapeB, direction));

	negate(direction);

	//Util::Vector a(1, 2, 3);
	//std::cout << (a*a) << std::endl;
	while (true) {
		simplex.push_back(support(_shapeA, _shapeB, direction));
		if (simplex[simplex.size() - 1]*direction<=0) {
			return false;
		}
		else {
			if (containsOrigin(simplex,direction)) {
				EPA(simplex, return_penetration_depth, return_penetration_vector, _shapeA, _shapeB);
				return true;
			}
		}
	}


	return false; // There is no collision
}

