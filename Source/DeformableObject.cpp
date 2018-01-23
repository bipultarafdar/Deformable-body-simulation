#include "DeformableObject.h"
#include <queue>

struct pointpair {
	vec3 cubepos;
	vec3 pos;
};

struct comparator {
	bool operator()(pointpair i, pointpair j) {
		return (i.cubepos != j.cubepos) && (length(i.cubepos) > length(j.cubepos));
	}
};

DeformableObject::DeformableObject() {
}

vec3 DeformableObject::lerp(vec3 start, vec3 end, float percent)
{
	return (start + percent*(end - start));
}

vec3 DeformableObject::trilerp(int i) {
	vec3 cubepos = model->vertices[i].cubePos;
	vec3 t = model->vertices[i].t;
	vec3 _000 = points[cubepos[0]][cubepos[1]][cubepos[2]].position;
	vec3 _100 = points[cubepos[0] + 1][cubepos[1]][cubepos[2]].position;
	vec3 _010 = points[cubepos[0]][cubepos[1] + 1][cubepos[2]].position;
	vec3 _001 = points[cubepos[0]][cubepos[1]][cubepos[2] + 1].position;
	vec3 _110 = points[cubepos[0] + 1][cubepos[1] + 1][cubepos[2]].position;
	vec3 _011 = points[cubepos[0]][cubepos[1] + 1][cubepos[2] + 1].position;
	vec3 _101 = points[cubepos[0] + 1][cubepos[1]][cubepos[2] + 1].position;
	vec3 _111 = points[cubepos[0] + 1][cubepos[1] + 1][cubepos[2] + 1].position;

	vec3 _00 = lerp(_000, _100, t[0]);
	vec3 _01 = lerp(_001, _101, t[0]);
	vec3 _10 = lerp(_010, _110, t[0]);
	vec3 _11 = lerp(_001, _111, t[0]);

	vec3 _0 = lerp(_00, _10, t[1]);
	vec3 _1 = lerp(_01, _11, t[1]);

	vec3 res = lerp(_0, _1, t[2]);

	return res;
}

void DeformableObject::updateModel() {
	for (int i = 0; i < model->vertices.size(); i++) {
		model->vertices[i].pos = trilerp(i);
	}
}

void DeformableObject::resolve(vec3 cubePos, Plane* p, vec3 pos) {
	points[cubePos[0]][cubePos[1]][cubePos[2]].collisionResolution(p, pos);
}


void DeformableObject::collisionResolution(Plane* p, int n) {
	bool flag = false;
	priority_queue<pointpair, std::vector<pointpair>, comparator> minHeap;
	for (int i = 0; i < model->vertices.size(); i++) {
		vec3 cubePos = model->vertices[i].cubePos;
		vec3 pos = model->vertices[i].pos;
		bool temp1 = p->checkCollisionWithPoint(model->vertices[i].pos);
		if (temp1 && !flag) {
			int arr[] = { 0, 1 };
			float min = std::numeric_limits<float>::max();
			vec3 point;
			vec3 temp = cubePos;
			float total = 0;
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 2; j++) {
					for (int k = 0; k < 2; k++) {
						int tempindex = i + 2 * (j + 2 * k);
						temp = cubePos + vec3(i, j, k);
						vec3 pos1 = points[temp[0]][temp[1]][temp[2]].position;
						if (p->checkCollisionWithPoint(pos1)) {
							flag = true;
							pointpair p;
							p.cubepos = temp;
							p.pos = pos1;
							minHeap.push(p);
						}
					}
				}
			}
		}
		else {
			vec3 temp = cubePos;
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 2; j++) {
					for (int k = 0; k < 2; k++) {
						int tempindex = i + 2 * (j + 2 * k);
						temp = cubePos + vec3(i, j, k);
						points[temp[0]][temp[1]][temp[2]].resetGravity();
					}
				}
			}
		}
	}
	int l = 0;
	while (l != n) {
		if (!minHeap.empty()) {
			pointpair a = minHeap.top();
			minHeap.pop();
			resolve(a.cubepos, p, a.pos);
		}
		l++;
	}
}

void DeformableObject::updatePosition(double deltaT) {
	for (int i = 0; i < numOfPoints; i++) {
		for (int j = 0; j < numOfPoints; j++) {
			for (int k = 0; k < numOfPoints; k++) {
				points[i][j][k].position += points[i][j][k].velocity * deltaT +
					(((float)0.5 * points[i][j][k].cacc) + points[i][j][k].ncacc) * deltaT * deltaT;
				points[i][j][k].velocity += (points[i][j][k].cacc + points[i][j][k].ncacc) * deltaT;
			}
		}
	}
}

vec3 getPos(Tmat4<float> a, vec4 b) {
	vec4 result(0);
	for (int m = 0; m < 3; m++) {
		for (int n = 0; n < 3; n++) {
			result[n] += b[m] * a[n][m];
		}
	}
	return{ result[0], result[1], result[2] };
}

DeformableObject::DeformableObject(int n, double space, string m, bool r, vec3 translate) : numOfPoints(n), spacing(space) 
{
	for (int i = 0; i < numOfPoints; i++) {
		vector<vector<DeformablePoint>> temp1;
		for (int j = 0; j < numOfPoints; j++) {
			vector<DeformablePoint> temp2;
			for (int k = 0; k < numOfPoints; k++) {
				float angle = 30;
				vec3 pos2;
				if(r){
					vec3 pos = getPos(vmath::rotate(angle, vmath::vec3(1, 0, 0)), vec4(spacing*i, spacing*j, spacing*k, 1));
					pos2 = getPos(vmath::rotate(angle, vmath::vec3(0, 0, 1)), vec4(pos, 1));
				}
				else {
					pos2 = vec3(spacing*i, spacing*j, spacing*k);
				}
				temp2.push_back(DeformablePoint(pos2 + translate));
			}
			temp1.push_back(temp2);
		}
		points.push_back(temp1);
	}

	int arr[2] = { -1, 1 };
	int arr2[2] = { -3, 3 };

	for (int i = 0; i < numOfPoints; i++) {
		for (int j = 0; j < numOfPoints; j++) {
			for (int k = 0; k < numOfPoints; k++) {
				DeformablePoint pt = points[i][j][k];
				//shear - start
				for (int t = 0; t < 2; t++) {
					for (int s = 0; s < 2; s++) {
						if (j + arr[t] >= 0 && j + arr[t] < numOfPoints && k + arr[s] >= 0 && k + arr[s] < numOfPoints) {
							pt.shearNeighbors.push_back(points[i][j + arr[t]][k + arr[s]]);
						}
					}
				}
				for (int p = 0; p < 2; p++) {
					for (int s = 0; s < 2; s++) {
						if (i + arr[p] >= 0 && i + arr[p] < numOfPoints && k + arr[s] >= 0 && k + arr[s] < numOfPoints) {
							pt.shearNeighbors.push_back(points[i + arr[p]][j][k + arr[s]]);
						}
					}
				}
				for (int p = 0; p < 2; p++) {
					for (int t = 0; t < 2; t++) {
						if (i + arr[p] >= 0 && i + arr[p] < numOfPoints && j + arr[t] >= 0 && j + arr[t] < numOfPoints) {
							pt.shearNeighbors.push_back(points[i + arr[p]][j + arr[t]][k]);
						}
					}
				}
				for (int p = 0; p < 2; p++) {
					for (int t = 0; t < 2; t++) {
						for (int s = 0; s < 2; s++) {
							if (i + arr[p] >= 0 && i + arr[p] < numOfPoints && j + arr[t] >= 0 && j + arr[t] < numOfPoints
								&& k + arr[s] >= 0 && k + arr[s] < numOfPoints) {
								pt.shearNeighbors.push_back(points[i + arr[p]][j + arr[t]][k + arr[s]]);
							}
						}
					}
				}
				//shear - end
				//structure - start
				for (int s = 0; s < 2; s++) {
					if (k + arr[s] >= 0 && k + arr[s] < numOfPoints) {
						pt.structuralNeighbors.push_back(points[i][j][k + arr[s]]);
					}
				}
				for (int t = 0; t < 2; t++) {
					if (j + arr[t] >= 0 && j + arr[t] < numOfPoints) {
						pt.structuralNeighbors.push_back(points[i][j + arr[t]][k]);
					}
				}
				for (int p = 0; p < 2; p++) {
					if (i + arr[p] >= 0 && i + arr[p] < numOfPoints) {
						pt.structuralNeighbors.push_back(points[i + arr[p]][j][k]);
					}
				}
				//structure - end
				//bending - start
				for (int s = 0; s < 2; s++) {
					if (k + arr2[s] >= 0 && k + arr2[s] < numOfPoints) {
						pt.bendingNeighbors.push_back(points[i][j][k + arr2[s]]);
					}
				}
				for (int t = 0; t < 2; t++) {
					if (j + arr2[t] >= 0 && j + arr2[t] < numOfPoints) {
						pt.bendingNeighbors.push_back(points[i][j + arr2[t]][k]);
					}
				}
				for (int p = 0; p < 2; p++) {
					if (i + arr2[p] >= 0 && i + arr2[p] < numOfPoints) {
						pt.bendingNeighbors.push_back(points[i + arr2[p]][j][k]);
					}
				}
				//bending - end
			}
		}
	}

	shearDist = length(points[0][0][0].position - points[1][0][1].position);
	shearAcrossDist = length(points[0][0][0].position - points[1][1][1].position);
	structureDist = length(points[0][0][0].position - points[0][0][1].position);
	bendingDist = structureDist * 3;
	loadModel(m);
}

void DeformableObject::loadModel(string m) {
	string filename = m + ".obj";
	model = new Model(spacing, numOfPoints, filename);
}


DeformableObject::~DeformableObject()
{
}
