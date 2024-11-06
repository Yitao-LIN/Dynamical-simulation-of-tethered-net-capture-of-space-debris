#pragma once
#include<iostream>
#include<Eigen/Dense>
#include"Object.h"
#include<queue>
#include"BroadPhaseCD.h"

using namespace std;
using namespace Eigen;
using namespace OBJ;

struct PAIRINFO {
	Vector2i Index;
	double distance;
	Vector3d RelativeVelocity;
};
struct CONTACTPAIR {
	double maxpenetration;
	Matrix<PAIRINFO, Dynamic, 1> Node2Node;
	Matrix<PAIRINFO, Dynamic, 1> Node2Face;
	CONTACTPAIR() {
		Node2Node.resize(0, 1);
		Node2Face.resize(0, 1);
	}
};

CONTACTPAIR NarrowPhaseCD(const NET &net, const TARGET &target, double TimeStep, double d_hat, Matrix<int, Dynamic, 2> PotentialCollisionPair);
double Distance(Vector3d vector1, Vector3d vector2);
double Distance(Vector3d vector1, Vector3d normal, Vector3d facevertice);
bool is_intriangle(Matrix3d FaceVertices, Vector3d FaceNormal, Vector3d vector1, double distance);
double calculate_area(Vector3d v1, Vector3d v2);