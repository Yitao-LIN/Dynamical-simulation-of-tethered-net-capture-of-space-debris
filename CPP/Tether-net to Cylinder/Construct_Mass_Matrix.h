#pragma once
#include<iostream>
#include<Eigen/Dense>
using namespace std;
using namespace Eigen;
VectorXd Construct_Mass_Matrix(int row, int column,  int n_node, MatrixXi connect_info, double m_tether, double corner_mass);
