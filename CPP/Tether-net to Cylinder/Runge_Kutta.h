#pragma once
#include<iostream>
#include<Eigen/Eigen>
#include"Node.h"
using namespace std;
using namespace Eigen;
NodeState Runge_Kutta(NodeState NodeStat, double step, int n_node, int n_edge, MatrixXi connect_info, VectorXd Mass_Matrix, double stiffness, double c, double len_tether, target_geo Target, VectorXd radius);