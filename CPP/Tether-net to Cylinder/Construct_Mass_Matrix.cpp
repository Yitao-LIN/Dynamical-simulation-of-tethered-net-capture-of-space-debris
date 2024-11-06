#include<iostream>
#include"Construct_Mass_Matrix.h"
using namespace std;

VectorXd Construct_Mass_Matrix(int row, int column, int n_node, MatrixXi connect_info, double m_tether, double corner_mass) {

	VectorXd Mass_Matrix(n_node);
	Mass_Matrix.setZero(n_node);

	for (int num_of_connection = 0; num_of_connection < connect_info.rows(); num_of_connection++) {
		int node_1 = connect_info(num_of_connection, 0) - 1;
		int node_2 = connect_info(num_of_connection, 1) - 1;
		Mass_Matrix(node_1) += m_tether / 2;
		Mass_Matrix(node_2) += m_tether / 2;
	}

	// corner mass
	for (int corner_i = 0; corner_i < 4; corner_i++) {
		Mass_Matrix(row * column + corner_i) = corner_mass;
	}
	return Mass_Matrix;
}