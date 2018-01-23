#pragma once

#include "DeformablePoint.h"
#include <vector>
#include "Model.h"

using namespace std;

class DeformableObject
{
public:
	int numOfPoints;
	double shearDist;
	double shearAcrossDist;
	double structureDist;
	double bendingDist;
	vector<vector<vector<DeformablePoint>>> points;
	Model* model;
	double spacing;
	vec3 lerp(vec3 start, vec3 end, float percent);
	vec3 trilerp(int i);
	void updateModel();
	void collisionResolution(Plane* p, int n);
	void resolve(vec3 cubePos, Plane* p, vec3 pos);
	void updatePosition(double deltaT);
	void loadModel(string m);
	DeformableObject();
	~DeformableObject();
	DeformableObject(int numOfPoints, double spacing, string m, bool r, vec3 translate);
};

