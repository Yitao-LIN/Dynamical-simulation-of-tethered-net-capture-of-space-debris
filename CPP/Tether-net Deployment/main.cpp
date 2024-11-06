#include<iostream>
#include<Eigen/Dense>
#include"Object.h"
#include"Runge_Kutta.h"
#include<math.h>
using namespace std;
using namespace Eigen;

int main() {
	int num_col = 15;
	int num_row = 15;
	double tether_initial_length = 0.25;
	double tether_stiffness = 12566.0;
	double single_tether_mass = 0.000784;
	double bullet_mass = 0.3;
	OBJ::NET net(num_col, num_row, tether_initial_length, tether_stiffness);
	net.Construct_ConnectInfo(net);
	net.Construct_MassVector(net, single_tether_mass, bullet_mass);
	net.AllocateNodeLocation(net);
	net.NetSettings(net);
	//cout << target.LocalPosition << endl;
	//cout << endl;
	double TimeStep = 0.0001;
	double EndTime = 3.0;
	double TimeProgress = 0;
	double d_hat = 0.000001;
	net.c = 0.2;
	/*
	cout << target.MeshInfo << endl;
	cout << endl;
	cout << target.FaceNormal << endl;
	cout << endl;
	cout << target.LocalPosition << endl;*/
	ofstream netpos;
	ofstream netacc;
	ofstream netvel;
	ofstream netlabel;
	ofstream mass;
	//cout << target.MeshInfo << endl;
	netlabel.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/0. Deployment Simulation/output/netlabel.txt");
	netlabel << net.ConnectInfo << endl;
	mass.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/0. Deployment Simulation/output/mass.txt");
	mass << net.MassVector << endl;

	netpos.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/0. Deployment Simulation/output/netpos.txt", ios::app);
	netacc.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/0. Deployment Simulation/output/netacc.txt", ios::app);
	netvel.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/0. Deployment Simulation/output/netvel.txt", ios::app);

	int fcout_calculator = 0;

	/*
	Vector3d ini_corner_position_1 = net.Configuration[net.NodeScale(2) - 1].Position;
	Vector3d ini_corner_position_2 = net.Configuration[net.NodeScale(2) - 2].Position;
	Vector3d ini_corner_position_3 = net.Configuration[net.NodeScale(2) - 3].Position;
	Vector3d ini_corner_position_4 = net.Configuration[net.NodeScale(2) - 4].Position;*/


	while (TimeProgress < EndTime)
	{
		TimeProgress += TimeStep;
		//if (TimeProgress >= 0.02) {
		//	cout << "check" << endl;
		//}

		Runge_Kutta(net, TimeStep, d_hat, netacc, fcout_calculator);
		/*
		net.Configuration[net.NodeScale(2) - 1].Position = ini_corner_position_1;
		net.Configuration[net.NodeScale(2) - 2].Position = ini_corner_position_2;
		net.Configuration[net.NodeScale(2) - 3].Position = ini_corner_position_3;
		net.Configuration[net.NodeScale(2) - 4].Position = ini_corner_position_4;
		net.Configuration[net.NodeScale(2) - 1].Velocity << 0, 0, 0;
		net.Configuration[net.NodeScale(2) - 2].Velocity << 0, 0, 0;
		net.Configuration[net.NodeScale(2) - 3].Velocity << 0, 0, 0;
		net.Configuration[net.NodeScale(2) - 4].Velocity << 0, 0, 0;*/
		/*
		for (int i = 0; i < net.NodeScale(2); ++i) {
			netpos << net.Configuration[i].Position.transpose() << endl;
			netvel << net.Configuration[i].Velocity.transpose() << endl;
		}
		netacc << target.CenterConfiguration.Position.transpose() << " " << target.CenterConfiguration.EulerAngle.transpose() << endl;
		*/

		
		if (fcout_calculator == 0) {
			for (int i = 0; i < net.NodeScale(2); ++i) {
				netpos << net.Configuration[i].Position.transpose() << endl;
				netvel << net.Configuration[i].Velocity.transpose() << endl;
			}
		}
		else if (fcout_calculator == 99) {
			fcout_calculator = -1;
		}
		++fcout_calculator;
		
	}


	return 0;
}