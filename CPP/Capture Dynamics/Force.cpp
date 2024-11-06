#include<iostream>
#include"Force.h"
#include<iomanip>
#include"BroadPhaseCD.h"
#include"NarrowPhaseCD.h"
#include"Object.h"

using namespace std;
using namespace Eigen;
using namespace OBJ;
MatrixXd Calculate_force(NET net, TARGET target, CONTACTPAIR& NarrowPhaseContactPair) {
	// collision stiffness and damping ratio //
	double k = 2.43E9;
	double d = 3315;

	// adaptive k d
	double maxpenetration = NarrowPhaseContactPair.maxpenetration;
	k *= sqrt(2 * abs(maxpenetration) / 3);
	d = 2 * 1.0 * sqrt(0.0012 * k);
	
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
	/*
	for (int node = 0; node < net.NodeScale(2); node++) {
		Force(node, 2) += (double) - net.MassVector(node) * 9.8;
	}*/

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////// add contact force term //////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////

	//cout << NarrowPhaseContactPair.Node2Face.row(0) << endl;
	if (NarrowPhaseContactPair.Node2Face.rows() > 0) {
		double friction_factor = 0.2;
		for (int pair_i = 0; pair_i < NarrowPhaseContactPair.Node2Face.rows(); pair_i++) {
			Vector3d Fn, Ft, M;
			ContactForceAndMoment(NarrowPhaseContactPair.Node2Face[pair_i], net, target, k, d, friction_factor, Fn, Ft, M);
			cout << NarrowPhaseContactPair.Node2Face[pair_i].distance << endl;
			Force.row(NarrowPhaseContactPair.Node2Face[pair_i].Index(0)) -= Fn;
			Force.row(Force.rows() - 1) += Fn;
			
			Moment += M;
		}
	}
	
	return Force;
}

void ContactForceAndMoment(PAIRINFO PairInfo, NET net, TARGET target, double k, double d, double friction_factor, Vector3d &Fn, Vector3d &Ft, Vector3d &M) {
	Vector3d FaceNormal = EulerRotationMatrix(target.CenterConfiguration.EulerAngle) * target.FaceNormal.row(PairInfo.Index(1)).transpose();
	Vector3d FaceTangential;
	if (net.Configuration[PairInfo.Index(0)].Velocity.norm() == 0) {
		Ft << 0, 0, 0;
	}
	else {
		FaceTangential = FaceNormal.cross((net.Configuration[PairInfo.Index(0)].Velocity).cross(FaceNormal));
		Ft = Fn.norm() * friction_factor * FaceTangential;
	}

	Fn = PairInfo.distance * k * FaceNormal;
	Fn += PairInfo.RelativeVelocity.dot(FaceNormal) * FaceNormal * d;
	
	Vector3d ContactPoint;
	ContactPoint = net.Configuration[PairInfo.Index(0)].Position - FaceNormal * net.NodeRadius;
	Vector3d Arm = ContactPoint - target.CenterConfiguration.Position;
	M = Arm.cross(Ft);
	if (isnan(Fn(0)) || isnan(Ft(0))) {
		cout << "nan" << endl;
	}
};
