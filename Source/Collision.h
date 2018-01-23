#pragma once

#include "vmath.h"

using namespace vmath;
using namespace std;

class Plane {
public:
	vec3 point;
	vec3 normal;
	Plane() {};
	~Plane() {};
	Plane(vec3 p, vec3 n) : point(p) {
		normal = normalize(n);
	};
	bool checkCollisionWithPoint( vec3 p ) {
		if (dot(point - p, normal) >= 0)
			return true;
		return false;
	}
};

class Sphere {
public:
	vec3 centre;
	float radius;
	Sphere();
	~Sphere();
	Sphere(vec3 c, float r) : centre(c), radius(r) {};
	bool checkCollisionWithPoint(vec3 p) {
		if (distance(centre, p) <= radius)
			return true;
		return false;
	}
};