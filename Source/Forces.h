#pragma once

#include "DeformableObject.h"
#include "vmath.h"

using namespace vmath;

class Forces {
public:
	//F = k*x^2
	double shear = 1; //spring const
	double structure = shear; //spring const
	double bending = shear; // spring const
	double damping = shear/3;

	DeformableObject* object;

	Forces(DeformableObject* object) : object(object) {};

	vec3 getSpringForce(DeformablePoint p1, DeformablePoint p2, double spConst, double originalDist) {
		vec3 distVec = (p2.position - p1.position);
		vec3 dir = normalize(distVec);
		double diff = length(distVec) - originalDist;
		double dampingForce = - damping * dot(p2.velocity - p1.velocity, dir);
		double springForce = spConst * diff;
		//if (diff < 0.000001) return (float)(springForce) * dir;
		return (float)(springForce - dampingForce) * dir;
	}

	void updateForces() {
		int arr[2] = { -1, 1 };
		int arr2[2] = { -3, 3 };
		for (int i = 0; i < object->numOfPoints; i++) {
			for (int j = 0; j < object->numOfPoints; j++) {
				for (int k = 0; k < object->numOfPoints; k++) {
					vec3 springForce(0);
					DeformablePoint p1 = object->points[i][j][k];
					//shear - start
					for (int t = 0; t < 2; t++) {
						for (int s = 0; s < 2; s++) {
							if (j + arr[t] >= 0 && j + arr[t] < object->numOfPoints && k + arr[s] >= 0 && k + arr[s] < object->numOfPoints) {
								springForce = springForce + getSpringForce(p1,
									object->points[i][j + arr[t]][k + arr[s]], shear, object->shearDist);
							}
						}
					}
					for (int p = 0; p < 2; p++) {
						for (int s = 0; s < 2; s++) {
							if (i + arr[p] >= 0 && i + arr[p] < object->numOfPoints && k + arr[s] >= 0 && k + arr[s] < object->numOfPoints) {
								springForce = springForce + getSpringForce(p1,
									object->points[i + arr[p]][j][k + arr[s]], shear, object->shearDist);
							}
						}
					}
					for (int p = 0; p < 2; p++) {
						for (int t = 0; t < 2; t++) {
							if (i + arr[p] >= 0 && i + arr[p] < object->numOfPoints && j + arr[t] >= 0 && j + arr[t] < object->numOfPoints) {
								springForce = springForce + getSpringForce(p1,
									object->points[i + arr[p]][j + arr[t]][k], shear, object->shearDist);
							}
						}
					}
					for (int p = 0; p < 2; p++) {
						for (int t = 0; t < 2; t++) {
							for (int s = 0; s < 2; s++) {
								if (i + arr[p] >= 0 && i + arr[p] < object->numOfPoints && j + arr[t] >= 0 && j + arr[t] < object->numOfPoints
									&& k + arr[s] >= 0 && k + arr[s] < object->numOfPoints) {
									springForce = springForce + getSpringForce(p1,
										object->points[i + arr[p]][j + arr[t]][k + arr[s]], shear, object->shearAcrossDist);
								}
							}
						}
					}
					//shear - end
					//structure - start
					for (int s = 0; s < 2; s++) {
						if (k + arr[s] >= 0 && k + arr[s] < object->numOfPoints) {
							springForce = springForce + getSpringForce(p1,
								object->points[i][j][k + arr[s]], structure, object->structureDist);
						}
					}
					for (int t = 0; t < 2; t++) {
						if (j + arr[t] >= 0 && j + arr[t] < object->numOfPoints) {
							springForce = springForce + getSpringForce(p1,
								object->points[i][j + arr[t]][k], structure, object->structureDist);
						}
					}
					for (int p = 0; p < 2; p++) {
						if (i + arr[p] >= 0 && i + arr[p] < object->numOfPoints) {
							springForce = springForce + getSpringForce(p1,
								object->points[i + arr[p]][j][k], structure, object->structureDist);
						}
					}
					//structure - end
					//bending - start
					for (int s = 0; s < 2; s++) {
						if (k + arr2[s] >= 0 && k + arr2[s] < object->numOfPoints) {
							springForce = springForce + getSpringForce(p1,
								object->points[i][j][k + arr2[s]], structure, object->bendingDist);
						}
					}
					for (int t = 0; t < 2; t++) {
						if (j + arr2[t] >= 0 && j + arr2[t] < object->numOfPoints) {
							springForce = springForce + getSpringForce(p1,
								object->points[i][j + arr2[t]][k], structure, object->bendingDist);
						}
					}
					for (int p = 0; p < 2; p++) {
						if (i + arr2[p] >= 0 && i + arr2[p] < object->numOfPoints) {
							springForce = springForce + getSpringForce(p1,
								object->points[i + arr2[p]][j][k], structure, object->bendingDist);
						}
					}
					object->points[i][j][k].ncacc = springForce;
					
				}
			}
		}

		/*cout <<"pos: " << object->points[0][0][0].position[0] << ", " << object->points[0][0][0].position[1] << ", " << object->points[0][0][0].position[2] << endl;
		cout <<"vel: "<< object->points[0][0][0].velocity[0] << ", " << object->points[0][0][0].velocity[1] << ", " << object->points[0][0][0].velocity[2] << endl;
		cout <<"acc: "<< object->points[0][0][0].acc[0] << ", " << object->points[0][0][0].acc[1] << ", " << object->points[0][0][0].acc[2] << endl;
		*/
	}

};