#pragma once
#include "vmath.h"

using namespace vmath;

class Point
{
public:
	vec3 position;
	Point(vec3 vec) : position(vec) {};
	Point() {};
	~Point() {};
};

