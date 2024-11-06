#pragma once
#include<iostream>
#include<Eigen/Dense>
#include"Node.h"
#include"Contact_detection.h"
using namespace std;
using namespace Eigen;
MatrixXd Calculate_force(NodeState NodeStat, int n_edge, int n_node, MatrixXi connect_info, double len_tether, double stiffness, double c, target_geo Target, VectorXd radius);
