#include<iostream>
#include<math.h>
#include"Contact_detection.h"
using namespace std;
using namespace Eigen;
MatrixXd Contact_detection(NodeState NodeStat, target_geo Target, int n_node, VectorXd radius) {
	MatrixXd contact_list(n_node, 5);
	contact_list.setZero();
	int number_of_contact_pairs = 0;
	// -1 for top;   -2 for bottom;   -3 for side face
	for (int i_node = 0; i_node < n_node; i_node++) {
		
		// first detection (top)
		double d_i = sqrt(pow(NodeStat.position(i_node, 0) - Target.Center_cyl(0), 2) + pow(NodeStat.position(i_node, 1) - Target.Center_cyl(1), 2)) - radius(i_node);

		if (NodeStat.position(i_node, 2) > Target.Center_cyl(2) 
			&& d_i < (Target.R_cyl - radius(i_node))
			&& NodeStat.position(i_node, 2) < Target.Center_cyl(2) + Target.H_cyl / 2.0 + radius(i_node)) {
			// pair number in _01
			contact_list(number_of_contact_pairs, 0) = -1;
			contact_list(number_of_contact_pairs, 1) = (double)i_node;
			// normal vector in _234
			contact_list(number_of_contact_pairs, 2) = 0;
			contact_list(number_of_contact_pairs, 3) = 0;
			contact_list(number_of_contact_pairs, 4) = -1.0;
			number_of_contact_pairs++;
		}

		if (NodeStat.position(i_node, 2) < Target.Center_cyl(2)
			&& d_i < (Target.R_cyl - radius(i_node))
			&& NodeStat.position(i_node, 2) > Target.Center_cyl(2) - Target.H_cyl / 2.0 - radius(i_node)) {
			// pair number in _01
			contact_list(number_of_contact_pairs, 0) = -2;
			contact_list(number_of_contact_pairs, 1) = (double)i_node;
			// normal vector in _234
			contact_list(number_of_contact_pairs, 2) = 0;
			contact_list(number_of_contact_pairs, 3) = 0;
			contact_list(number_of_contact_pairs, 4) = 1.0;
			number_of_contact_pairs++;
		}

		if (Target.Center_cyl(2) - Target.H_cyl / 2 < NodeStat.position(i_node, 2)
			&& Target.Center_cyl(2) + Target.H_cyl / 2 > NodeStat.position(i_node, 2)
			&& d_i < Target.R_cyl) {
			if (contact_list(number_of_contact_pairs, 0) == -1 && abs(d_i - Target.R_cyl) < abs(NodeStat.position(i_node, 2) -( Target.Center_cyl(2) + Target.H_cyl / 2.0 + radius(i_node)))) {
				// pair number in _01
				contact_list(number_of_contact_pairs, 0) = -3;
				contact_list(number_of_contact_pairs, 1) = (double)i_node;
				// normal vector in _234
				contact_list(number_of_contact_pairs, 2) = (NodeStat.position(i_node, 0) - Target.Center_cyl(0)) / (d_i + radius(i_node));
				contact_list(number_of_contact_pairs, 3) = (NodeStat.position(i_node, 1) - Target.Center_cyl(1)) / (d_i + radius(i_node));
				contact_list(number_of_contact_pairs, 4) = 0;
				//cout << contact_list(number_of_contact_pairs, 2) << " " << contact_list(number_of_contact_pairs, 3) << " " << sqrt(pow(contact_list(number_of_contact_pairs, 2), 2) + pow(contact_list(number_of_contact_pairs, 3), 2)) << endl;

				number_of_contact_pairs++;
			}
			else if (contact_list(number_of_contact_pairs, 0) == -2 && abs(d_i - Target.R_cyl) < abs(NodeStat.position(i_node, 2) - (Target.Center_cyl(2) - Target.H_cyl / 2.0 - radius(i_node)))) {
				// pair number in _01
				contact_list(number_of_contact_pairs, 0) = -3;
				contact_list(number_of_contact_pairs, 1) = (double)i_node;
				// normal vector in _234
				contact_list(number_of_contact_pairs, 2) = (NodeStat.position(i_node, 0) - Target.Center_cyl(0)) / (d_i + radius(i_node));
				contact_list(number_of_contact_pairs, 3) = (NodeStat.position(i_node, 1) - Target.Center_cyl(1)) / (d_i + radius(i_node));
				contact_list(number_of_contact_pairs, 4) = 0;
				//cout << contact_list(number_of_contact_pairs, 2) << " " << contact_list(number_of_contact_pairs, 3) << " " << sqrt(pow(contact_list(number_of_contact_pairs, 2), 2) + pow(contact_list(number_of_contact_pairs, 3), 2)) << endl;
				number_of_contact_pairs++;
			}
			else if (contact_list(number_of_contact_pairs, 0) == 0) {
				// pair number in _01
				contact_list(number_of_contact_pairs, 0) = -3;
				contact_list(number_of_contact_pairs, 1) = (double)i_node;
				// normal vector in _234
				contact_list(number_of_contact_pairs, 2) = (NodeStat.position(i_node, 0) - Target.Center_cyl(0)) / (d_i + radius(i_node));
				contact_list(number_of_contact_pairs, 3) = (NodeStat.position(i_node, 1) - Target.Center_cyl(1)) / (d_i + radius(i_node));
				contact_list(number_of_contact_pairs, 4) = 0;
				//cout << contact_list(number_of_contact_pairs, 2) << " " << contact_list(number_of_contact_pairs, 3) << " " << sqrt(pow(contact_list(number_of_contact_pairs, 2), 2) + pow(contact_list(number_of_contact_pairs, 3), 2)) << endl;
				number_of_contact_pairs++;
			}
		}
	}
	
	return contact_list;
}