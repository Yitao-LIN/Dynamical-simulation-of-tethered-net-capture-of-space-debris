#include<iostream>
#include "Node.h"

using namespace std;

NodeState Construct_Nodes(int row, int column, double len_tether, VectorXd mass) {
	// Initialize nodal position and other information
	// put this tether net in the x-y coordinate surface
	// the first node (ball 0) will be placed at (0, 0, 0)

	double n_node = row * column;

	NodeState nodeStat;
	nodeStat.position.setZero(n_node + 4, 3);
	nodeStat.velocity.setZero(n_node + 4, 3);
	nodeStat.acceleration.setZero(n_node + 4, 3);
	//radius.setZero(n_node + 4);

	nodeStat.t = 0;

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			nodeStat.position((i * column + j), 0) = i * len_tether;
			nodeStat.position((i * column + j), 1) = j * len_tether;
			nodeStat.position((i * column + j), 2) = 0;
			//radius(i * column + j) = mass(i * column + j) * 0.01;
			/*
			nodeStat.velocity[(i * column + j) * 3] = 0;
			nodeStat.velocity[(i * column + j) * 3 + 1] = 0;
			nodeStat.velocity((i * column + j), 2) = 2.0;*/

			/*
			nodeStat.acceleration[(i * column + j) * 3] = 0;
			nodeStat.acceleration[(i * column + j) * 3 + 1] = 0;
			nodeStat.acceleration[(i * column + j) * 3 + 2] = 0;
			*/
		}
	}

	// corner mass 1
	nodeStat.position.row(n_node) = nodeStat.position.row(0);
	nodeStat.position(n_node, 0) += -len_tether * sqrt(2) / 2.0;
	nodeStat.position(n_node, 1) += -len_tether * sqrt(2) / 2.0;
	//radius(n_node) = mass(n_node) * 0.1;

	// corner mass 2
	nodeStat.position.row(n_node + 1) = nodeStat.position.row(column-1);
	nodeStat.position(n_node + 1, 0) += -len_tether * sqrt(2) / 2.0;
	nodeStat.position(n_node + 1, 1) += len_tether * sqrt(2) / 2.0;
	//radius(n_node + 1) = mass(n_node + 1) * 0.1;

	// corner mass 3
	nodeStat.position.row(n_node + 2) = nodeStat.position.row(n_node - column);
	nodeStat.position(n_node + 2, 0) += len_tether * sqrt(2) / 2.0;
	nodeStat.position(n_node + 2, 1) += -len_tether * sqrt(2) / 2.0;
	//radius(n_node + 2) = mass(n_node + 2) * 0.1;

	// corner mass 4
	nodeStat.position.row(n_node + 3) = nodeStat.position.row(n_node - 1);
	nodeStat.position(n_node + 3, 0) += len_tether * sqrt(2) / 2.0;
	nodeStat.position(n_node + 3, 1) += len_tether * sqrt(2) / 2.0;
	//radius(n_node + 3) = mass(n_node + 3) * 0.1;


	return nodeStat;
}