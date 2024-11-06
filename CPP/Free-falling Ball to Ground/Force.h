#pragma once
#include<iostream>
#include<Eigen/Dense>
using namespace std;
using namespace Eigen;
double Calculate_force(Vector4d NodeStat, double m, double g, double k, double d, double h);
