#pragma once
#include<iostream>
#include<Eigen/Dense>
using namespace std;
using namespace Eigen;

class NodeState {
public:
	MatrixXd position;
	MatrixXd velocity;
	MatrixXd acceleration;
	VectorXd radius;
	double t;
};

NodeState Construct_Nodes(int row, int column, double len_tether, VectorXd mass);
