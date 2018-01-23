#pragma once

#include <vector>
#include "vmath.h"
//#include "DeformableObject.h"
#include <fstream>
#include <sstream>
#include <array>
#include <cmath>
#define EPS 0.0000001

using namespace vmath;
using namespace std;

class Vertex {
public:
	vec3 pos;
	vec3 cubePos;
	vec3 t;
	Vertex(vec3 pos) : pos(pos) {};
};

class Model {
public:
	vector<Vertex> vertices; 
	vector<array<int, 3>> triangles;
	vec3 bmin = vec3(INFINITY, INFINITY, INFINITY), bmax = vec3(-INFINITY, -INFINITY, -INFINITY);
	Model() {};
	
	//might be faulty
	vec3 getPos(Tmat4<float> a, vec4 b) {
		vec4 result(0);
		for (int m = 0; m < 4; m++)	{
			for (int n = 0; n < 4; n++){
				result[n] += b[n] * a[n][m];
			}
		}
		return{ result[0], result[1], result[2] };
	}
	
	Model(float spacing, float numOfPoints, string& file) {
		load_obj(file);
		//find min and max point and compress to fit obj
		//Tmat4<float> trans = translate(-bmin);
		bmax = bmax - bmin;
		float t = std::max(bmax[0], std::max(bmax[1], bmax[2]));
		t = (numOfPoints - 1) * spacing / t;
		//Tmat4<float> sc = scale(t);
		float invSpacing = 1 / spacing;
		for (int i = 0; i < vertices.size(); i++) {
			vertices[i].pos = vertices[i].pos - bmin;
			vertices[i].pos = (float)(t - EPS) * vertices[i].pos;
			vertices[i].cubePos = vec3(floor(vertices[i].pos[0] * invSpacing), 
				floor(vertices[i].pos[1] * invSpacing), floor(vertices[i].pos[2] * invSpacing)); 
			vertices[i].t = (vertices[i].pos - vertices[i].cubePos) * invSpacing;
		}
	}


	void load_obj(std::string& filename) {
		std::ifstream in(filename);
		if (in.fail()) {
			std::cout << filename << std::endl;
			throw std::logic_error("Can't read the mesh filea at BaseMode::ReadObjFile().");
		}
		char buf[256];
		vmath::vec3 pt;
		std::array<string, 3> ft;
		while (in.getline(buf, sizeof buf)) {
			std::istringstream line(buf);
			std::string word;
			line >> word;
			if (word == "v") {
				line >> pt[0] >> pt[1] >> pt[2];
				vertices.push_back(Vertex(pt));
				for (int k = 0; k < 3; ++k) {
					bmin[k] = std::min(pt[k], bmin[k]);
					bmax[k] = std::max(pt[k], bmax[k]);
				}
			}
			else if (word == "f") {
				line >> ft[0] >> ft[1] >> ft[2];
				string delimiter = "/";
				array<int, 3> s;
				for (unsigned i = 0; i < 3; i++) {
					string token = ft[i].substr(0, ft[i].find(delimiter));
					s[i] = atoi(token.c_str());
					--s[i];
				}
				triangles.push_back(s);
			}
		}
		/*modelcenter_ = (bmin + bmax);
		modelcenter_ *= 0.5;
		vmath::vec3 diff = bmax - bmin;
		float scale = 1.2 / std::max(std::max(diff[0], diff[1]), diff[2]);
		modelscale_ = vmath::vec3(scale, scale, scale);
		in.close();
		normals_.resize(triangles_.size());
		for (unsigned i = 0; i < triangles_.size(); ++i) {
			vmath::vec3 ab = vertices_[triangles_[i][1]] - vertices_[triangles_[i][0]];
			vmath::vec3 ac = vertices_[triangles_[i][2]] - vertices_[triangles_[i][0]];
			normals_[i] = normalize(cross(ab, ac));
		}*/
	}
};