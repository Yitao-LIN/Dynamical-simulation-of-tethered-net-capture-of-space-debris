#pragma once
#include<iostream>
#include<Eigen/Dense>
#include"Object.h"
#include<queue>
using namespace std;
using namespace Eigen;
using namespace OBJ;

typedef Matrix<double, 2, 3, RowMajor> BOX;

struct LOCATIONLIST {
	pair<int, int> x;

	LOCATIONLIST(int a, int b) { x.first = a; x.second = b; }
	bool operator < (const LOCATIONLIST& a) const {
		return x.first > a.x.first;
	}
};

struct COLLISIONPAIRLIST {
	pair<int, int> x;
	COLLISIONPAIRLIST(int a, int b) { x.first = a; x.second = b; }
	bool operator < (const COLLISIONPAIRLIST& a) const {
		return x.first > a.x.first;
	}
};

typedef struct HASHGRID {
	BOX HashGridBoundary;
	Vector3i devide_num;
	Matrix<double, 1, Dynamic> HashGridInterval_X;
	Matrix<double, 1, Dynamic> HashGridInterval_Y;
	Matrix<double, 1, Dynamic> HashGridInterval_Z;
	priority_queue<LOCATIONLIST> BoundingBoxLocation;
public:
	HASHGRID(Vector3i devide, BOX boundary) {
		devide_num = devide;
		HashGridBoundary = boundary;
	}
	void CreatHashGridInterval(HASHGRID &HashGrid) {
		HashGridInterval_X.conservativeResize(1, devide_num(0)+1);
		for (int i = 0; i < devide_num(0); i++) {
			HashGridInterval_X(i) = HashGridBoundary(0, 0) + i * (HashGridBoundary(1, 0) - HashGridBoundary(0, 0)) / devide_num(0);
		}
		HashGridInterval_X(devide_num(0)) = HashGridBoundary(1, 0);
		HashGridInterval_Y.conservativeResize(1, devide_num(1) + 1);
		for (int i = 0; i < devide_num(1); i++) {
			HashGridInterval_Y(i) = HashGridBoundary(0, 1) + i * (HashGridBoundary(1, 1) - HashGridBoundary(0, 1)) / devide_num(1);
		}
		HashGridInterval_Y(devide_num(1)) = HashGridBoundary(1, 1);

		HashGridInterval_Z.conservativeResize(1, devide_num(2) + 1);
		for (int i = 0; i < devide_num(2); i++) {
			HashGridInterval_Z(i) = HashGridBoundary(0, 2) + i * (HashGridBoundary(1, 2) - HashGridBoundary(0, 2)) / devide_num(2);
		}
		HashGridInterval_Z(devide_num(2)) = HashGridBoundary(1, 2);
	}
};
Matrix3d EulerRotationMatrix(Vector3d EulerAngle);
bool is_contact(HASHGRID HashGrid, NET net, TARGET target, double TimeStep, double d_hat, Matrix<int, Dynamic, 2> &PotentialCollisionPair);
MatrixXd RodriguesMatrix(double Theta, Vector3d AxisDirection);
MatrixXd SkewMatrix(Vector3d AxisDirection);