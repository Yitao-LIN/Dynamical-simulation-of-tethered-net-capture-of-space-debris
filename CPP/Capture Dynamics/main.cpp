#include<iostream>
#include<Eigen/Dense>
#include"Object.h"
#include"Runge_Kutta.h"
#include<math.h>
#include"BroadPhaseCCD.h"
#include"NarrowPhaseCD.h"
using namespace std;
using namespace Eigen;

int main() {
	int num_col = 17;
	int num_row = 17;
	double tether_initial_length = 0.20;
	double tether_stiffness = 12566.0;
	double single_tether_mass = 0.000784;
	double bullet_mass = 1.0;
	const char* OBJNAME = "/objs/ball_origin.obj";
	OBJ::TARGET::CENTERCONFIG init_center_configuration;
	init_center_configuration.Position << 0, 0, 0.5;
	init_center_configuration.Velocity << 0, 0, 0;
	init_center_configuration.EulerAngle << 0, 0, 0;
	OBJ::NET net(num_col, num_row, tether_initial_length, tether_stiffness);
	OBJ::TARGET target(OBJNAME, init_center_configuration);
	net.Construct_ConnectInfo(net);
	net.Construct_MassVector(net, single_tether_mass, bullet_mass);
	net.AllocateNodeLocation(net);
	net.NetSettings(net);

	double TimeStep = 0.000001;
	double EndTime = 5.0;
	double TimeProgress = 0;
	double d_hat = 0.000001;
	net.c = 0.2;

	ofstream netpos;
	ofstream tarpos;
	ofstream netvel;
	ofstream netlabel;
	ofstream mass;
	//cout << target.MeshInfo << endl;
	netlabel.open("/output/netlabel.txt");
	netlabel << net.ConnectInfo << endl;
	mass.open("/output/mass.txt");
	mass << net.MassVector << endl;
	tarpos.open("/output/tarpos.txt");
	tarpos.clear();
	tarpos.close();

	netpos.open("/output/netpos.txt", ios::app);
	tarpos.open("/output/tarpos.txt", ios::app);
	netvel.open("/output/netvel.txt", ios::app);

	int fcout_calculator = 0;

	Vector3i devide_num(16, 16, 16);
	BOX HashGridBoundary;
	HashGridBoundary << -4, -4, -4, 4, 4, 4;
	HASHGRID HashGrid(devide_num, HashGridBoundary);
	HashGrid.CreatHashGridInterval(HashGrid);

	while (TimeProgress < EndTime)
	{
		TimeProgress += TimeStep;
		Runge_Kutta(net, target, TimeStep, d_hat, HashGrid);
		if (fcout_calculator == 0) {
			for (int i = 0; i < net.NodeScale(2); ++i) {
				netpos << net.Configuration[i].Position.transpose() << endl;
				netvel << net.Configuration[i].Velocity.transpose() << endl;
			}
			tarpos << target.CenterConfiguration.Position.transpose() << " " << target.CenterConfiguration.EulerAngle.transpose() << endl;
		}
		else if (fcout_calculator == 9) {
			fcout_calculator = -1;
		}
		++fcout_calculator;
	}


	return 0;
}