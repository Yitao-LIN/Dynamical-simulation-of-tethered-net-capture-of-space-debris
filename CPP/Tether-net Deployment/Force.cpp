#include<iostream>
#include"Force.h"
#include<iomanip>

#include"Object.h"

using namespace std;
using namespace Eigen;
using namespace OBJ;
MatrixXd Calculate_force(NET net) {
	// collision stiffness and damping ratio //
	double k = 2.43E9;
	double d = 3315;
	
	MatrixXd Force(net.NodeScale(2) + 1, 3);
	
	Force.setZero();
	Vector3d Moment;
	Moment.setZero();
	/*
	for (int i = 0; i < NarrowPhaseContactPair.Node2Face.rows(); i++) {
		cout << NarrowPhaseContactPair.Node2Face(i).Index.transpose() << endl;
	}*/
	// iteration goes through every edge
	for (int e = 0; e < net.EdgeScale; e++) {
		// confirm node number of chosen edge
		int node_i = net.ConnectInfo(e, 0) - 1;
		int node_j = net.ConnectInfo(e, 1) - 1;
		// get direction vector of two nodes and chosen edge
		Vector3d r_i = net.Configuration[node_i].Position.transpose();
		Vector3d r_j = net.Configuration[node_j].Position.transpose();
		Vector3d r_ij = r_j - r_i;
		// get direction cosine of chosen edge
		Vector3d cos_r_ij(0, 0, 0);
		cos_r_ij(0) = (double)r_ij(0) / r_ij.norm();
		cos_r_ij(1) = (double)r_ij(1) / r_ij.norm();
		cos_r_ij(2) = (double)r_ij(2) / r_ij.norm();

		//double len_ij = r_ij.norm();
		double len_ij = (double) sqrt(r_ij(0) * r_ij(0) + r_ij(1) * r_ij(1) + r_ij(2) * r_ij(2));
		// get the current force given by spring
		double temp_Force = (double) net.tether_stiffness * (len_ij - net.tether_initial_length);
		// add up with the force given by damper/ dash pot
		Vector3d v_ij = net.Configuration[node_j].Velocity.transpose() - net.Configuration[node_i].Velocity.transpose();
		temp_Force += (double) net.c * v_ij.dot(cos_r_ij);
		// tether net stiffness correction
		if (len_ij <= net.tether_initial_length) {
			temp_Force = (double) 0.0;
		}
		else {
			int a = 1;
		}

		// calculate coordinate component and fill them in the Force matrix
		Force.row(node_i) +=  temp_Force * cos_r_ij;
		Force.row(node_j) +=  -temp_Force * cos_r_ij;
		
		//cout << e << endl;
		//cout << Force << endl;
	}
	double airdamping = 0.2;
	/*
	for (int node = 0; node < net.NodeScale(2) - 4; node++) {
		//Force(node, 2) += (double) - net.MassVector(node) * 9.8;
		if (net.Configuration[node].Velocity(2) < 0) {
			Force.row(node) += net.Configuration[node].Velocity * net.Configuration[node].Velocity * airdamping * 1.29 * 3.14 * 0.001 * 0.001;
		}
		else {
			Force.row(node) += -net.Configuration[node].Velocity * net.Configuration[node].Velocity * airdamping * 1.29 * 3.14 * 0.001 * 0.001;
		}
	}*/
	return Force;
}