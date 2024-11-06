#pragma once
#include<iostream>
#include<Eigen/Dense>
using namespace std;
using namespace Eigen;

Vector4d Runge_Kutta(Vector4d NodeStat, double step, double m, double g, double k, double d, double h);