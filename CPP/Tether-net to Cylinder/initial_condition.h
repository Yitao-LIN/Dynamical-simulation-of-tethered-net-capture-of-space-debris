#pragma once
#include<iostream>
#include"Node.h"
#include<Eigen/Dense>
using namespace std;
using namespace Eigen;

NodeState Corner_Z_velocity_mode(NodeState NodeStat, double velocity, int n_row, int n_col);
