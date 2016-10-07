#include "obstacles/GJK_EPA.h"
#define TOLERANCE 0.000001

SteerLib::GJK_EPA::GJK_EPA()
{
}

float area(const std::vector<Util::Vector>& _shape) {
	int len = _shape.size();
	float result = 0;
	for (int i = len - 1, j = 0; j < len; i = j, j++) {
		result += _shape[i].x * _shape[j].z - _shape[j].x * _shape[i].z;
	}
	return result * 0.5;
}
bool inside_help(const std::vector<Util::Vector>& _shape, std::vector<int>& vertices, int u, int v, int w, int index){
	float ux = _shape[vertices[w]].x - _shape[vertices[v]].x;
	float uz = _shape[vertices[w]].z - _shape[vertices[v]].z;
	float vx = _shape[vertices[u]].x - _shape[vertices[w]].x;
	float vz = _shape[vertices[u]].z - _shape[vertices[w]].z;
	float wx = _shape[vertices[v]].x - _shape[vertices[u]].x;
	float wz = _shape[vertices[v]].z - _shape[vertices[u]].z;
	float upx = _shape[vertices[index]].x - _shape[vertices[u]].x;
	float upz = _shape[vertices[index]].z - _shape[vertices[u]].z;
	float vpx = _shape[vertices[index]].x - _shape[vertices[v]].x;
	float vpz = _shape[vertices[index]].z - _shape[vertices[v]].z;
	float wpx = _shape[vertices[index]].x - _shape[vertices[w]].x;
	float wpz = _shape[vertices[index]].z - _shape[vertices[w]].z;
	float uxvp = ux*vpz - uz*vpx;
	float wxup = wx*upz - wz*upx;
	float vxwp = vx*wpz - vz*wpx;
	return (uxvp >= 0 && wxup >= 0 && vxwp >= 0);
}
bool inside(const std::vector<Util::Vector>& _shape, int u, int v, int w, int n, std::vector<int>& vertices) {
	int index;
	float u_x, u_z, v_x, v_z, w_x, w_z, index_x, index_z;
	u_x = _shape[vertices[u]].x;
	u_z = _shape[vertices[u]].z;
	v_x = _shape[vertices[v]].x;
	v_z = _shape[vertices[v]].z;
	w_x = _shape[vertices[w]].x;
	w_z = _shape[vertices[w]].z;
	if (TOLERANCE > (((v_x - u_x) *(w_z - u_z)) - ((v_z - u_z) * (w_x - u_x)))) return false;
	for (index = 0; index < n; index++) {
		if (index == u || index == v || index == w) continue;
		index_x = _shape[vertices[index]].x;
		index_z = _shape[vertices[index]].z;
		if (inside_help(_shape, vertices, u, v, w, index)) return false;
	}
	return true;
}
bool decomposite(const std::vector<Util::Vector>& _shape, std::vector<std::vector<Util::Vector>>& dcomp_A) {
	int len = _shape.size();
	if (len < 3) return false;
	std::vector<int> vertices(len);
	if (0.0 < area(_shape)) {
		for (int i = 0; i < len; i++) vertices[i] = i;
	}else {
		for (int i = 0; i < len; i++) vertices[i] = len - i - 1;
	}
	int size_v = len;
	int count = 2 * size_v;
	for (int i = 0, v = size_v - 1; size_v > 2;) {
		if (0 >= (count--)) return false;
		int u = v;
		if (size_v <= u) u = 0;
		v = u + 1;
		if (size_v <= v) v = 0;
		int w = v + 1;
		if (size_v <= w) w = 0;
		if (inside(_shape, u, v, w, size_v, vertices)) {
			int a, b, c;
			a = vertices[u];
			b = vertices[v];
			c = vertices[w];
			std::vector<Util::Vector> tmp;
			tmp.push_back(_shape[a]);
			tmp.push_back(_shape[b]);
			tmp.push_back(_shape[c]);
			dcomp_A.push_back(tmp);
			i++;
			for (int j = v, k = v + 1; k < size_v; j++, k++)
				vertices[j] = vertices[k];
			size_v--;
			count = 2 * size_v;
			std::cout << count<<std::endl;
		}
	}
	std::cout << "success" << std::endl;
	return true;
}

bool on_segment(Util::Vector A, Util::Vector B)
{
	Util::Vector AB = B - A;
	Util::Vector AO = -1 * A;
	float rx = 0, ry = 0, rz = 0;
	bool flag_x = false;
	bool flag_y = false;
	bool flag_z = false; 
	if (AB.x == 0 || AO.x == 0)
	{
		if (!(AB.x == 0 && AO.x == 0)) return false; 
		flag_x = true;
	}else{
		rx = AO.x / AB.x;
	}
	if (AB.y == 0 || AO.y == 0)
	{
		if (!(AB.y == 0 && AO.y == 0)) return false; 
		flag_y = true;
	}else{
		ry = AO.y / AB.y;
	}
	if (AB.z == 0 || AO.z == 0)
	{
		if (!(AB.z == 0 && AO.z == 0))  return false; 
		flag_z = true;
	}else{
		rz = AO.z / AB.z;
	}
	std::vector<float> r;
	if (!flag_x) r.push_back(rx);
	if (!flag_y) r.push_back(ry);
	if (!flag_z) r.push_back(rz);

	if (r.size() == 3)
	{
		if (r[0] == r[1] && r[1] == r[2])
		{
			if (0 <= r[0] && r[0] <= 1) return true;
		}
		else return false;
	}
	else if (r.size() == 2)
	{
		if (r[0] == r[1])
		{
			if (0 <= r[0] && r[0] <= 1) return true;
		}
		else return false;

	}
	else if (r.size() == 1)
	{
		if (0 <= r[0] && r[0] <= 1) return true;
		else return false;
	}
	else return true;
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

		if (on_segment(A, B) || on_segment(A, C)) return true;
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
	if (!isConvex(_shapeA) || !isConvex(_shapeB)) {
		std::vector<std::vector<Util::Vector>> dcomp_A;
		std::vector<std::vector<Util::Vector>> dcomp_B;
		if (decomposite(_shapeA, dcomp_A) && decomposite(_shapeB, dcomp_B)) {
			std::pair<float, Util::Vector> max_f;
			max_f.first = FLT_MIN;
			for (int i = 0; i < dcomp_A.size(); i++) {
				for (int j = 0; j < dcomp_B.size(); j++) {
					std::pair<float, Util::Vector> tmp;
					if (intersect(tmp.first, tmp.second, dcomp_A[i], dcomp_B[j])) {
						if (max_f.first < tmp.first) {
							max_f.first = tmp.first;
							max_f.second = tmp.second;
						}
					}
				}
			}
			if (max_f.first != FLT_MIN) {
				return_penetration_depth = max_f.first;
				return_penetration_vector = max_f.second;
				return true;
			}
			else return false;
		}
	}

	Util::Vector zero(0, 0, 0);
	Util::Vector start_a(0, 0, 0), start_b(0, 0, 0);
	std::vector<Util::Vector> simplex;
	Util::Vector direction;
	direction = getCenter(_shapeB) - getCenter(_shapeA);

	simplex.push_back(support(_shapeA, _shapeB, direction));

	negate(direction);
	while (true) {
		simplex.push_back(support(_shapeA, _shapeB, direction));
		if (direction == zero)
		{
			//std::cout << " direction == Origin" << std::endl;
			std::vector<Util::Vector>::iterator it = simplex.begin();
			Util::Vector p1(it->x, it->y, it->z);
			it++;
			Util::Vector p2(it->x, it->y, it->z);
			if (on_segment(p1, p2))
			{
				EPA(simplex, return_penetration_depth, return_penetration_vector, _shapeA, _shapeB);
				return true;
			}
			else
				return false;
		}
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

