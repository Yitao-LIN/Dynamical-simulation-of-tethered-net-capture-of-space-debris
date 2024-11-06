#pragma once
#include<iostream>
#include"Node.h"
using namespace std;
using namespace Eigen;

class target_geo {
public:
	double R_cyl;
	double H_cyl;
	Vector3d Center_cyl;
};

MatrixXd Contact_detection(NodeState NodeStat, target_geo Target, int n_node, VectorXd radius);