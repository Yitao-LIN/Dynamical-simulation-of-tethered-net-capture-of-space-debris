#include<iostream>
#include"Node.h"
#include"Force.h"
using namespace std;
using namespace Eigen;
NodeState Runge_Kutta(NodeState NodeStat, double step, int n_node, int n_edge, MatrixXi connect_info, VectorXd Mass_Matrix, double stiffness, double c, double len_tether, target_geo Target , VectorXd radius) {
	MatrixXd tempForce;
	// NodeStat update
	tempForce = Calculate_force(NodeStat, n_edge, n_node, connect_info, len_tether, stiffness, c, Target, radius);
	for (int node = 0; node < n_node; node++) {
		NodeStat.acceleration.row(node) = tempForce.row(node) / Mass_Matrix(node);
	}

	// velocity
	MatrixXd f_1_1(n_node, 3); f_1_1.setZero();
	// acceleration
	MatrixXd f_1_2(n_node, 3); f_1_2.setZero();
	for (int node = 0; node < n_node; node++) {
		f_1_1.row(node) += step * NodeStat.velocity.row(node);
		f_1_2.row(node) += step * NodeStat.acceleration.row(node);
	}

	// velocity
	MatrixXd f_2_1(n_node, 3); f_2_1.setZero();
	// acceleration
	MatrixXd f_2_2(n_node, 3); f_2_2.setZero();
	// state + 1/2 f_1
	NodeState tempStat;
	tempStat.position.setZero(n_node, 3);
	tempStat.velocity.setZero(n_node, 3);
	tempStat.acceleration.setZero(n_node, 3);
	

	for (int node = 0; node < n_node; node++) {
		tempStat.position.row(node) = NodeStat.position.row(node) + f_1_1.row(node) / 2.0;
		tempStat.velocity.row(node) = NodeStat.velocity.row(node) + f_1_2.row(node) / 2.0;
	}
	tempForce = Calculate_force(tempStat, n_edge, n_node, connect_info, len_tether, stiffness, c, Target, radius);
	for (int node = 0; node < n_node; node++) {
		tempStat.acceleration.row(node) = tempForce.row(node) / Mass_Matrix(node);
	}
	for (int node = 0; node < n_node; node++) {
		f_2_1.row(node) += step * tempStat.velocity.row(node);
		f_2_2.row(node) += step * tempStat.acceleration.row(node);
	}
	

	// velocity
	MatrixXd f_3_1(n_node, 3); f_3_1.setZero();
	// acceleration
	MatrixXd f_3_2(n_node, 3); f_3_2.setZero();
	// state + 1/2 f_2
	for (int node = 0; node < n_node; node++) {
		tempStat.position.row(node) = NodeStat.position.row(node) + f_2_1.row(node) / 2.0;
		tempStat.velocity.row(node) = NodeStat.velocity.row(node) + f_2_2.row(node) / 2.0;
	}
	tempForce = Calculate_force(tempStat, n_edge, n_node, connect_info, len_tether, stiffness, c, Target, radius);
	for (int node = 0; node < n_node; node++) {
		tempStat.acceleration.row(node) = tempForce.row(node) / Mass_Matrix(node);
	}
	for (int node = 0; node < n_node; node++) {
		f_3_1.row(node) += step * tempStat.velocity.row(node);
		f_3_2.row(node) += step * tempStat.acceleration.row(node);
	}

	// velocity
	MatrixXd f_4_1(n_node, 3); f_4_1.setZero();
	// acceleration
	MatrixXd f_4_2(n_node, 3); f_4_2.setZero();
	// state + f_3
	for (int node = 0; node < n_node; node++) {
		tempStat.position.row(node) = NodeStat.position.row(node) + f_3_1.row(node);
		tempStat.velocity.row(node) = NodeStat.velocity.row(node) + f_3_2.row(node);
	}
	tempForce = Calculate_force(tempStat, n_edge, n_node, connect_info, len_tether, stiffness, c, Target, radius);
	for (int node = 0; node < n_node; node++) {
		tempStat.acceleration.row(node) = tempForce.row(node) / Mass_Matrix(node);
	}
	for (int node = 0; node < n_node; node++) {
		f_4_1.row(node) += step * tempStat.velocity.row(node);
		f_4_2.row(node) += step * tempStat.acceleration.row(node);
	}

	// newstat = state + 1/6( f1 + 2*f2 + 2*f3 + f4 )
	for (int node = 0; node < n_node; node++) {
		NodeStat.position.row(node) += (f_1_1.row(node) + 2.0 * f_2_1.row(node) + 2.0 * f_3_1.row(node) + f_4_1.row(node)) / 6.0;
		NodeStat.velocity.row(node) += (f_1_2.row(node) + 2.0 * f_2_2.row(node) + 2.0 * f_3_2.row(node) + f_4_2.row(node)) / 6.0;
	}

	//cout << f_1_1 + 2 * f_2_1 + 2 * f_3_1 + f_4_1 << endl;
	NodeStat.t += step;
	return NodeStat;
}