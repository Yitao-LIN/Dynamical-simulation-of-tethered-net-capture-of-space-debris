#include<iostream>
#include"Force.h"
#include<iomanip>
using namespace std;
using namespace Eigen;
double Calculate_force(Vector4d NodeStat, double m, double g, double k, double d, double h) {
	double Force = 0;
	if (NodeStat(0) < 0.0) {
		Force = (double)m * g - NodeStat(0) * k - NodeStat(1) * d;
	}
	else{
		Force = (double)m * g;
	}
	return Force;
}
