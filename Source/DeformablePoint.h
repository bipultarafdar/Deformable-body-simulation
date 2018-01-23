#pragma once

#include "vmath.h"
#include <vector>
#include "Collision.h"
#include <iostream>

using namespace std;
using namespace vmath;

class DeformablePoint
{
public:
	float invmass = 5;
	float e = 0.9;
	vec3 force = vec3(0);
	vec3 position;
	vec3 velocity = vec3(0);
	vec3 gravity = vec3(0, -0.2, 0);
	vec3 cacc = gravity; //acceleration;
	vec3 ncacc = vec3(0);
	vector<DeformablePoint> shearNeighbors;
	vector<DeformablePoint> shearNeighborsAcross;
	vector<DeformablePoint> structuralNeighbors;
	vector<DeformablePoint> bendingNeighbors;
	DeformablePoint() {};
	~DeformablePoint() {};
	DeformablePoint(vec3 vec) { position = vec; }

	//change to penalty force
	void collisionResolution(Plane* p, vec3 pos) {
		//vec3 t = - abs(dot(velocity, p->normal)) * p->normal;
		//velocity = velocity - (1 + e) * t;
		float x = dot(pos - p->point, p->normal);
		//cacc = vec3(0);
		ncacc -= (float)20 * x * p->normal;
		velocity = e * velocity;
		//cacc = -cacc;
			//print(velocity);
			//cacc = vec3(0);
	}

	void print(vec3 a) {
		cout << a[0] << ", " << a[1] << ", " << a[0] << endl;
	}

	void resetGravity() {
		cacc = gravity;
	}

	void collisionResolution(Sphere s) {
		if (s.checkCollisionWithPoint(position)) {
			float a = 1 / length(velocity);
			velocity = e * dot(-velocity, position - s.centre) * a * velocity;
		}
	}
};

