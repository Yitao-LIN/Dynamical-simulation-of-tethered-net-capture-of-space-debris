#include<iostream>
#include<fstream>
#include<math.h>
#include<Eigen/Dense>
#include"Labeling.h"
#include"Construct_Mass_Matrix.h"
#include"Node.h"
#include"Force.h"
#include"Runge_Kutta.h"
#include"initial_condition.h"
#include"Contact_detection.h"


using namespace Eigen;
using namespace std;

int main() {
	// Chaser
	int n_row = 10;
	int n_col = 10;
	int n_edge = n_row * (n_col - 1) + n_col * (n_row - 1);
	int n_node = n_row * n_col + 4;
	double m_tether = 1.0;
	double corner_mass = 5.0;
	double l_tether = 0.5;
	double stiffness = 7000 * acos(-1);
	double c = 1.0;
	double step = 0.001;
	double total_time = 1;
	
	// Target
	// In this scenario, target is defined as a cylinder
	// the axis of cylinder perpendicular to the x-y plane
	target_geo Target;
	Target.R_cyl = 0.5;
	Target.H_cyl = 4.0;
	Target.Center_cyl(0) = (n_col - 1) * l_tether / 2.0;
	Target.Center_cyl(1) = (n_row - 1) * l_tether / 2.0;
	Target.Center_cyl(2) = 2.1;

	// output files
	ofstream postxt;
	ofstream veltxt;
	ofstream acctxt;
	ofstream label;
	ofstream mass;
	ofstream radiustxt;

	label.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/label.txt");
	mass.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/mass.txt");
	radiustxt.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/radius.txt");

	postxt.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/position.txt");
	postxt.clear();
	postxt.close();
	veltxt.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/velocity.txt");
	veltxt.clear();
	veltxt.close();
	acctxt.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/acceleration.txt");
	acctxt.clear();
	acctxt.close();

	MatrixXi connect_info(n_edge, 2);
	connect_info = Labeling(n_row, n_col, n_edge);
	label << connect_info;
	//cout << connect_info;
	VectorXd Mass_Matrix(n_node);
	Mass_Matrix = Construct_Mass_Matrix(n_row, n_col, n_node, connect_info, m_tether, corner_mass);
	mass << Mass_Matrix;
	//cout << Mass_Matrix;
	NodeState NodeStat;
	NodeStat = Construct_Nodes(n_row, n_col, l_tether, Mass_Matrix);
	NodeStat = Corner_Z_velocity_mode(NodeStat, 5.0, n_row, n_col);
	//cout << NodeStat.position;
	VectorXd radius(n_node);

	for (int i_node = 0; i_node < n_node; i_node++) {
		radius(i_node) = Mass_Matrix(i_node) * 0.01;
	}
	radiustxt << radius;

	postxt.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/position.txt", ios:: app);
	veltxt.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/velocity.txt", ios::app);
	acctxt.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/2Tether_net_to_TriMesh/output/acceleration.txt", ios::app);

	while (NodeStat.t < total_time) {
		NodeStat = Runge_Kutta(NodeStat, step, n_node, n_edge, connect_info, Mass_Matrix, stiffness, c, l_tether, Target, radius);
		postxt << NodeStat.position << endl;
		veltxt << NodeStat.velocity << endl;
		acctxt << NodeStat.acceleration << endl;
	}
	return 0;
}