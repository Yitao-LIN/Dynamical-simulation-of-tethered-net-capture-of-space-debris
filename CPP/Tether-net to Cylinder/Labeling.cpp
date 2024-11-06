#include<iostream>
#include "Labeling.h"

MatrixXi Labeling(int n_row, int n_col, int n_edge) {
	
	MatrixXi connect_info(n_edge + 4, 2);
	double n_node = n_row * n_col;

	int node_1 = 0;
	int node_2 = 0;
	int edge_count = 0;

	for (int i = 0; i < n_row;  i++) {
		node_1 = i * n_row + 1;
		node_2 = i * n_row + 2;
		for (int j = 0; j < n_col-1; j++) {
			connect_info(edge_count, 0) = node_1;
			connect_info(edge_count, 1) = node_2;

			edge_count++;
			node_1++;
			node_2++;
		}
	}

	for (int i = 0; i < n_row-1; i++) {
		node_1 = i * n_col + 1;
		for (int j = 0; j < n_col; j++) {
			connect_info(edge_count, 0) = node_1;
			connect_info(edge_count, 1) = node_1 + n_row;

			edge_count++;
			node_1++;
		}
	}
	
	// corner label
	connect_info(edge_count, 0) = 1;
	connect_info(edge_count, 1) = n_node + 1;
	edge_count++;

	connect_info(edge_count, 0) = n_col;
	connect_info(edge_count, 1) = n_node + 2;
	edge_count++;

	connect_info(edge_count, 0) = n_node - n_col + 1;
	connect_info(edge_count, 1) = n_node + 3;
	edge_count++;

	connect_info(edge_count, 0) = n_node;
	connect_info(edge_count, 1) = n_node + 4;

	return connect_info;
}
