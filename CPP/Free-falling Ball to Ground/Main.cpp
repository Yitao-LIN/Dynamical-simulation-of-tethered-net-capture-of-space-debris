#include<iostream>
#include<fstream>
#include<Eigen/Dense>
#include"Runge_Kutta.h"
using namespace std;
using namespace Eigen;

int main() {
	double m = 1.0;
	double g = -9.8;
	double k = 100000.0;
	double d = 1.0;
	double h = 1.0;
	Vector4d NodeStat;
	NodeStat.setZero();
	NodeStat(0) = h;
	double step = 0.001;
	ofstream position;
	ofstream velocity;
	ofstream acceleration;
	position.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/Mass_Spring_Damped_0319/output2/position.txt");
	velocity.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/Mass_Spring_Damped_0319/output2/velocity.txt");
	acceleration.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/Mass_Spring_Damped_0319/output2/acceleration.txt");
	while (NodeStat(3) < 5.0) {
		NodeStat = Runge_Kutta(NodeStat, step, m, g, k, d, h);
		position << NodeStat(0) << endl;
		velocity << NodeStat(1) << endl;
		acceleration << NodeStat(2) << endl;
	}
	return 0;
}