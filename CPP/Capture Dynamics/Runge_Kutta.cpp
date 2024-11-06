#include<iostream>
#include"Runge_Kutta.h"
#include<queue>
using namespace std;
using namespace Eigen;
using namespace OBJ;

void Runge_Kutta(NET &net, TARGET &target, double TimeStep, double d_hat, HASHGRID &HashGrid) {
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////// DO CONTACT DETECTION ///////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	Matrix<int, Dynamic, 2> PotentialCollisionPair;
	PotentialCollisionPair.resize(0, 2);
	bool is_broad_contact = is_contact(HashGrid, net, target, TimeStep, d_hat, PotentialCollisionPair);
	
	CONTACTPAIR NarrowPhaseContactPair;
	if (is_broad_contact) {
		NarrowPhaseContactPair = NarrowPhaseCD(net, target, TimeStep, d_hat, PotentialCollisionPair);
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////// END CONTACT DETECTION ////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

	MatrixXd tempForce;
	// NodeStat update
	tempForce = Calculate_force(net, target, NarrowPhaseContactPair);

	// velocity
	MatrixXd f_1_1(net.NodeScale(2) + 1, 3); f_1_1.setZero();
	// acceleration
	MatrixXd f_1_2(net.NodeScale(2) + 1, 3); f_1_2.setZero();
	//cout << endl;

	int node = 0;
	for (node = 0; node < net.NodeScale(2) + 1; ++node) {
		if (node != net.NodeScale(2)) {
			f_1_1.row(node) = net.Configuration[node].Velocity;
			f_1_2.row(node) = tempForce.row(node) / net.MassVector(node);
			//cout << net.Configuration[node].Velocity.transpose() << endl;

		}
		else {
			f_1_1.row(node) = target.CenterConfiguration.Velocity;
			f_1_2.row(node) = tempForce.row(node) / target.mass;
		}
	}
	// velocity
	MatrixXd f_2_1(net.NodeScale(2) + 1, 3); f_2_1.setZero();
	// acceleration
	MatrixXd f_2_2(net.NodeScale(2) + 1, 3); f_2_2.setZero();
	// state + 1/2 f_1

	// Use deep copy to avoid address conflict!!!!
	NET tempnet(net);
	TARGET temptarget(target);

	for (node = 0; node < net.NodeScale(2) + 1; ++node) {
		if (node != net.NodeScale(2)) {
			tempnet.Configuration[node].Position = net.Configuration[node].Position + TimeStep * f_1_1.row(node).transpose() / 2.0;
			tempnet.Configuration[node].Velocity = net.Configuration[node].Velocity + TimeStep * f_1_2.row(node).transpose() / 2.0;

		}
		else {
			temptarget.CenterConfiguration.Position = target.CenterConfiguration.Position + TimeStep * f_1_1.row(node).transpose() / 2.0;
			temptarget.CenterConfiguration.Velocity = target.CenterConfiguration.Velocity + TimeStep * f_1_2.row(node).transpose() / 2.0;
		}

	}
	tempForce = Calculate_force(tempnet, temptarget, NarrowPhaseContactPair);
	for (node = 0; node < tempnet.NodeScale(2) + 1; ++node) {
		if (node != net.NodeScale(2)) {
			f_2_1.row(node) += tempnet.Configuration[node].Velocity;
			f_2_2.row(node) += tempForce.row(node) / tempnet.MassVector(node);
		}
		else {
			f_2_1.row(node) += temptarget.CenterConfiguration.Velocity;
			f_2_2.row(node) += tempForce.row(node) / temptarget.mass;
		}
	}

	// velocity
	MatrixXd f_3_1(net.NodeScale(2) + 1, 3); f_3_1.setZero();
	// acceleration
	MatrixXd f_3_2(net.NodeScale(2) + 1, 3); f_3_2.setZero();
	// state + 1/2 f_2
	for (node = 0; node < net.NodeScale(2) + 1; ++node) {
		if (node != net.NodeScale(2)) {
			tempnet.Configuration[node].Position = net.Configuration[node].Position + TimeStep * f_2_1.row(node).transpose() / 2.0;
			tempnet.Configuration[node].Velocity = net.Configuration[node].Velocity + TimeStep * f_2_2.row(node).transpose() / 2.0;
		}
		else {
			temptarget.CenterConfiguration.Position = target.CenterConfiguration.Position + TimeStep * f_2_1.row(node).transpose() / 2.0;
			temptarget.CenterConfiguration.Velocity = target.CenterConfiguration.Velocity + TimeStep * f_2_2.row(node).transpose() / 2.0;
		}
	}

	tempForce = Calculate_force(tempnet, temptarget, NarrowPhaseContactPair);
	for (node = 0; node < tempnet.NodeScale(2) + 1; ++node) {
		if (node != net.NodeScale(2)) {
			f_3_1.row(node) += tempnet.Configuration[node].Velocity;
			f_3_2.row(node) += tempForce.row(node) / tempnet.MassVector(node);
		}
		else {
			f_3_1.row(node) += temptarget.CenterConfiguration.Velocity;
			f_3_2.row(node) += tempForce.row(node) / temptarget.mass;
		}
	}

	// velocity
	MatrixXd f_4_1(tempnet.NodeScale(2) + 1, 3); f_4_1.setZero();
	// acceleration
	MatrixXd f_4_2(tempnet.NodeScale(2) + 1, 3); f_4_2.setZero();
	// state + f_3
	//cout << endl;
	for (node = 0; node < net.NodeScale(2) + 1; ++node) {
		if (node != net.NodeScale(2)) {
			tempnet.Configuration[node].Position = net.Configuration[node].Position + TimeStep * f_3_1.row(node).transpose();
			tempnet.Configuration[node].Velocity = net.Configuration[node].Velocity + TimeStep * f_3_2.row(node).transpose();
			//cout << tempnet.Configuration[node].Position.transpose() << endl;
		}
		else {
			temptarget.CenterConfiguration.Position = target.CenterConfiguration.Position + TimeStep * f_3_1.row(node).transpose();
			temptarget.CenterConfiguration.Velocity = target.CenterConfiguration.Velocity + TimeStep * f_3_2.row(node).transpose();
		}
	}
	tempForce = Calculate_force(tempnet, temptarget, NarrowPhaseContactPair);
	//cout << tempForce << endl;
	for (node = 0; node < net.NodeScale(2) + 1; ++node) {
		if (node != net.NodeScale(2)) {
		f_4_1.row(node) += tempnet.Configuration[node].Velocity;
		f_4_2.row(node) += tempForce.row(node) / tempnet.MassVector(node);
		}
		else {
		f_4_1.row(node) += temptarget.CenterConfiguration.Velocity;
		f_4_2.row(node) += tempForce.row(node) / temptarget.mass;
		}
	}

	// newstat = state + 1/6( f1 + 2*f2 + 2*f3 + f4 )
	//cout << "updated configuration" << endl;
	for (node = 0; node < net.NodeScale(2) + 1; ++node) {
		if (node != net.NodeScale(2)) {
			net.Configuration[node].Position += TimeStep * (f_1_1.row(node) + 2.0 * f_2_1.row(node) + 2.0 * f_3_1.row(node) + f_4_1.row(node)) / 6.0;
			net.Configuration[node].Velocity += TimeStep * (f_1_2.row(node) + 2.0 * f_2_2.row(node) + 2.0 * f_3_2.row(node) + f_4_2.row(node)) / 6.0;
			//cout << net.Configuration[node].Velocity.transpose() << endl;
		}
		else {
			target.CenterConfiguration.Position += TimeStep * (f_1_1.row(node) + 2.0 * f_2_1.row(node) + 2.0 * f_3_1.row(node) + f_4_1.row(node)) / 6.0;
			target.CenterConfiguration.Velocity += TimeStep * (f_1_2.row(node) + 2.0 * f_2_2.row(node) + 2.0 * f_3_2.row(node) + f_4_2.row(node)) / 6.0;
		}
	}

	delete[] tempnet.Configuration;
	
}
