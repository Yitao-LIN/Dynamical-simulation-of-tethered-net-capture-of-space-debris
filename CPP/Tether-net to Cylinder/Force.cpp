#include<iostream>
#include"Force.h"
#include<iomanip>
using namespace std;
using namespace Eigen;
MatrixXd Calculate_force(NodeState NodeStat, int n_edge, int n_node, MatrixXi connect_info, double len_tether, double stiffness, double c, target_geo Target, VectorXd radius) {
	double k = 10000000.0;
	double d = 206.0;
	MatrixXd Force(n_node, 3);
	Force.setZero();
	// iteration goes through every edge
	for (int e = 0; e < n_edge + 4; e++) {
		// confirm node number of chosen edge
		int node_i = connect_info(e, 0)-1;
		int node_j = connect_info(e, 1)-1;
		// get direction vector of two nodes and chosen edge
		Vector3d r_i = NodeStat.position.row(node_i).transpose();
		Vector3d r_j = NodeStat.position.row(node_j).transpose();
		Vector3d r_ij = r_j - r_i;
		// get direction cosine of chosen edge
		Vector3d cos_r_ij(0, 0, 0);
		cos_r_ij(0) = (double)r_ij(0) / r_ij.norm();
		cos_r_ij(1) = (double)r_ij(1) / r_ij.norm();
		cos_r_ij(2) = (double)r_ij(2) / r_ij.norm();
		/*
		if (e == 0) {
			cout << cos_r_ij(0) << " " << cos_r_ij(1) << " " << cos_r_ij(2) << " ";
			if (cos_r_ij.norm() > 1) {
				double warn = (double)cos_r_ij.norm();
				cout <<  " Warning! " << setprecision(8) << warn;
			}
		}*/
		//Vector3d cos_r_ij(r_ij(0)/r_ij.norm(), r_ij(1) / r_ij.norm(), r_ij(2) / r_ij.norm());
		// get current length of chosen edge (gonna compare with its initial length)
		double len_ij = r_ij.norm();
		// get the current force given by spring
		double temp_Force = stiffness * (len_ij - len_tether);
		// add up with the force given by damper/ dash pot
		Vector3d v_ij = NodeStat.velocity.row(node_j).transpose() - NodeStat.velocity.row(node_i).transpose();
		temp_Force += c * v_ij.dot(cos_r_ij);
		// tether net stiffness correction
		if (len_ij < len_tether) {
			temp_Force = 0;
		}
		// calculate coordinate component and fill them in the Force matrix
		Force.row(node_i) += temp_Force * cos_r_ij;
		Force.row(node_j) += -temp_Force * cos_r_ij;
	}

	MatrixXd contact_list(n_node, 5);
	contact_list.setZero();
	contact_list = Contact_detection(NodeStat, Target, n_node, radius);
	//cout << contact_list << endl;
	bool is_contact = false;
	if (contact_list(0, 0) != contact_list(0, 1)) {
		for (int i_pair = 0; i_pair < contact_list.rows(); i_pair++) {
			is_contact = false;
			double delta_i;
			if (contact_list(i_pair, 0) == -1) {
				delta_i = NodeStat.position(contact_list(i_pair, 1), 2) - Target.Center_cyl(2) - radius(contact_list(i_pair, 1)) - Target.H_cyl / 2.0;
				is_contact = true;
			}
			else if (contact_list(i_pair, 0) == -2) {
				delta_i = Target.Center_cyl(2) - NodeStat.position(contact_list(i_pair, 1), 2) - radius(contact_list(i_pair, 1)) - Target.H_cyl / 2.0;
				is_contact = true;
			}
			else if (contact_list(i_pair, 0) == -3) {
				Vector2d coordinate_xyplane_node(NodeStat.position(contact_list(i_pair, 1), 0), NodeStat.position(contact_list(i_pair, 1), 1));
				Vector2d coordinate_xyplane_target(Target.Center_cyl(0), Target.Center_cyl(1));
				delta_i = Target.R_cyl + radius(contact_list(i_pair, 1)) - (coordinate_xyplane_node - coordinate_xyplane_target).norm();
				is_contact = true;
			}
			if (is_contact) {
				Vector3d u_n_unit;
				u_n_unit << contact_list(i_pair, 2), contact_list(i_pair, 3), contact_list(i_pair, 4);
				double delta_i_dot = (NodeStat.velocity(contact_list(i_pair, 1)) * u_n_unit).norm();
				Force.row(contact_list(i_pair, 1)) += k * delta_i * u_n_unit + d * delta_i_dot * u_n_unit;
				u_n_unit.swap(u_n_unit);
				//cout << contact_list.row(i_pair) << " " << delta_i << " " << delta_i_dot << endl;
				//Force(contact_list(i_pair, 1), 1) += k * delta_i * contact_list(i_pair, 3);
				//Force(contact_list(i_pair, 1), 2) += k * delta_i * contact_list(i_pair, 4);
			}
		}
		//cout << " " << endl;
	}
	
	//delete contact_list;

	//cout << Force << endl;
	return Force;
}
