#include<iostream>
#include"Force.h"
using namespace std;
using namespace Eigen;

Vector4d Runge_Kutta(Vector4d NodeStat, double step, double m, double g, double k, double d, double h) {
	double tempForce;
	// NodeStat update
	tempForce = Calculate_force(NodeStat, m, g, k, d, h);
	NodeStat(2) = tempForce / m;

	Vector2d f_1; f_1.setZero();
	Vector2d f_2; f_2.setZero();
	Vector2d f_3; f_3.setZero();
	Vector2d f_4; f_4.setZero();

	f_1(0) = step * NodeStat(1);
	f_1(1) = step * NodeStat(2);

	Vector4d tempStat;

	tempStat.setZero();
	tempStat(0) = NodeStat(0) + f_1(0) / 2.0;
	tempStat(1) = NodeStat(1) + f_1(1) / 2.0;
	tempForce = Calculate_force(tempStat, m, g, k, d, h);
	tempStat(2) = tempForce / m;
	f_2(0) = step * tempStat(1);
	f_2(1) = step * tempStat(2);
	
	tempStat.setZero();
	tempStat(0) = NodeStat(0) + f_2(0) / 2.0;
	tempStat(1) = NodeStat(1) + f_2(1) / 2.0;
	tempForce = Calculate_force(tempStat, m, g, k, d, h);
	tempStat(2) = tempForce / m;
	f_3(0) = step * tempStat(1);
	f_3(1) = step * tempStat(2);

	tempStat.setZero();
	tempStat(0) = NodeStat(0) + f_3(0);
	tempStat(1) = NodeStat(1) + f_3(1);
	tempForce = Calculate_force(tempStat, m, g, k, d, h);
	tempStat(2) = tempForce / m;
	f_4(0) = step * tempStat(1);
	f_4(1) = step * tempStat(2);

	// newstat = state + 1/6( f1 + 2*f2 + 2*f3 + f4 )
	NodeStat(0) += (f_1(0) + 2.0 * f_2(0) + 2.0 * f_3(0) + f_4(0)) / 6.0;
	NodeStat(1) += (f_1(1) + 2.0 * f_2(1) + 2.0 * f_3(1) + f_4(1)) / 6.0;
	//cout << f_1_1 + 2 * f_2_1 + 2 * f_3_1 + f_4_1 << endl;
	NodeStat(3) += step;
	return NodeStat;
}