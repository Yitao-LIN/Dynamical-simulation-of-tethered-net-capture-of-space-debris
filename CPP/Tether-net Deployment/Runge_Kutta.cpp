#include<iostream>
#include"Runge_Kutta.h"
#include<queue>
using namespace std;
using namespace Eigen;
using namespace OBJ;



void Runge_Kutta(NET &net, double TimeStep, double d_hat, ofstream& netacc, int fcout_calculator) {
	MatrixXd tempForce;
	// NodeStat update
	tempForce = Calculate_force(net);
	if (fcout_calculator == 0) {
		netacc << tempForce << endl;
	}
	// velocity
	MatrixXd f_1_1(net.NodeScale(2), 3); f_1_1.setZero();
	// acceleration
	MatrixXd f_1_2(net.NodeScale(2), 3); f_1_2.setZero();
	int node = 0;
	for (node = 0; node < net.NodeScale(2); ++node) {
		f_1_1.row(node) = net.Configuration[node].Velocity;
		f_1_2.row(node) = tempForce.row(node) / net.MassVector(node);
	}

	// velocity
	MatrixXd f_2_1(net.NodeScale(2), 3); f_2_1.setZero();
	// acceleration
	MatrixXd f_2_2(net.NodeScale(2), 3); f_2_2.setZero();
	// state + 1/2 f_1
	// 引用的是地址，因此tempnet改变，net也改变，将大错特错！
	NET tempnet(net);
	for (node = 0; node < net.NodeScale(2); ++node) {
		tempnet.Configuration[node].Position = net.Configuration[node].Position + TimeStep * f_1_1.row(node).transpose() / 2.0;
		tempnet.Configuration[node].Velocity = net.Configuration[node].Velocity + TimeStep * f_1_2.row(node).transpose() / 2.0;
	}
	tempForce = Calculate_force(tempnet);
	for (node = 0; node < tempnet.NodeScale(2); ++node) {
		f_2_1.row(node) += tempnet.Configuration[node].Velocity;
		f_2_2.row(node) += tempForce.row(node) / tempnet.MassVector(node);
	}


	// velocity
	MatrixXd f_3_1(net.NodeScale(2), 3); f_3_1.setZero();
	// acceleration
	MatrixXd f_3_2(net.NodeScale(2), 3); f_3_2.setZero();
	// state + 1/2 f_2
	for (node = 0; node < net.NodeScale(2); ++node) {
		tempnet.Configuration[node].Position = net.Configuration[node].Position + TimeStep * f_2_1.row(node).transpose() / 2.0;
		tempnet.Configuration[node].Velocity = net.Configuration[node].Velocity + TimeStep * f_2_2.row(node).transpose() / 2.0;
	}

	tempForce = Calculate_force(tempnet);
	for (node = 0; node < tempnet.NodeScale(2); ++node) {
		f_3_1.row(node) += tempnet.Configuration[node].Velocity;
		f_3_2.row(node) += tempForce.row(node) / tempnet.MassVector(node);
	}
	// velocity
	MatrixXd f_4_1(tempnet.NodeScale(2), 3); f_4_1.setZero();
	// acceleration
	MatrixXd f_4_2(tempnet.NodeScale(2), 3); f_4_2.setZero();
	// state + f_3
	for (node = 0; node < net.NodeScale(2); ++node) {
		tempnet.Configuration[node].Position = net.Configuration[node].Position + TimeStep * f_3_1.row(node).transpose();
		tempnet.Configuration[node].Velocity = net.Configuration[node].Velocity + TimeStep * f_3_2.row(node).transpose();
	}
	tempForce = Calculate_force(tempnet);
	for (node = 0; node < net.NodeScale(2); ++node) {
		f_4_1.row(node) += tempnet.Configuration[node].Velocity;
		f_4_2.row(node) += tempForce.row(node) / tempnet.MassVector(node);
	}
	
	// newstat = state + 1/6( f1 + 2*f2 + 2*f3 + f4 )
	//cout << "updated configuration" << endl;
	for (node = 0; node < net.NodeScale(2); ++node) {
		net.Configuration[node].Position += TimeStep * (f_1_1.row(node) + 2.0 * f_2_1.row(node) + 2.0 * f_3_1.row(node) + f_4_1.row(node)) / 6.0;
		net.Configuration[node].Velocity += TimeStep * (f_1_2.row(node) + 2.0 * f_2_2.row(node) + 2.0 * f_3_2.row(node) + f_4_2.row(node)) / 6.0;
		//cout << net.Configuration[node].Velocity.transpose() << endl;
	}

	delete[] tempnet.Configuration;
}
