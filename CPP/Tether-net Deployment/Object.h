#pragma once
#include<iostream>
#include<Eigen/Dense>
#include<fstream>
#include<vector>

using namespace std;
using namespace Eigen;

namespace OBJ{
//////////////////////////////////////////////////////////////////////////////
// Construct class to represent a lumped-parameter tether net
//////////////////////////////////////////////////////////////////////////////

class NET {
public:
	class NODECONFIG {
	public:
		Vector3d Position;
		Vector3d Velocity;
	};
public:
	const double NodeRadius = 0.001;
	Vector3i NodeScale;
	double EdgeScale;
	MatrixXd ConnectInfo;
	VectorXd MassVector;
	NODECONFIG* Configuration;
	double tether_initial_length;
	double tether_stiffness;
	double c;
	//bool Construct_ConnectInfo(NET &net);
	//void Construct_MassVector(NET &net,  double single_tether_mass, double bullet_mass);

	NET(const int num_col, const int num_row, double tether_initial_length, double tether_stiffness) {
		NodeScale << num_col, num_row, num_col* num_row + 4;
		ConnectInfo.conservativeResize(num_col * (num_row - 1) + num_row * (num_col - 1)+4, 2);
		ConnectInfo.setZero();
		MassVector.conservativeResize(NodeScale(2));
		Configuration = new NODECONFIG[NodeScale(2)];
		this->tether_initial_length = tether_initial_length;
		this->tether_stiffness = tether_stiffness;
	}

	// 增加一个拷贝构造函数
	NET(const NET& originnet) {
		this->NodeScale = originnet.NodeScale;
		this->EdgeScale = originnet.EdgeScale;
		this->ConnectInfo = originnet.ConnectInfo;
		this->c = originnet.c;
		this->MassVector = originnet.MassVector;
		this->tether_initial_length = originnet.tether_initial_length;
		this->tether_stiffness = originnet.tether_stiffness;
		// 动态数组需要深拷贝
		this->Configuration = new NODECONFIG[NodeScale(2)];
		for (int deepcopy_i = 0; deepcopy_i < NodeScale(2); deepcopy_i++) {
			this->Configuration[deepcopy_i].Position = originnet.Configuration[deepcopy_i].Position;
			this->Configuration[deepcopy_i].Velocity = originnet.Configuration[deepcopy_i].Velocity;
		}
	}

	bool Construct_ConnectInfo(NET &net) {
		int edge_count = 0;
		for (int i = 0; i < NodeScale(1); i++) {
			int node_1 = i * NodeScale(1) + 1;
			int node_2 = i * NodeScale(1) + 2;
			for (int j = 0; j < NodeScale(0) - 1; j++) {
				ConnectInfo(edge_count, 0) = node_1;
				ConnectInfo(edge_count, 1) = node_2;
				edge_count++;
				node_1++;
				node_2++;
			}
		}
		for (int i = 0; i < NodeScale(1) - 1; i++) {
			int node_1 = i * NodeScale(0) + 1;
			for (int j = 0; j < NodeScale(0); j++) {
				ConnectInfo(edge_count, 0) = node_1;
				ConnectInfo(edge_count, 1) = node_1 + NodeScale(1);
				edge_count++;
				node_1++;
			}
		}
		// corner label
		ConnectInfo(edge_count, 0) = 1;
		ConnectInfo(edge_count, 1) = NodeScale(2) - 3;
		edge_count++;
		ConnectInfo(edge_count, 0) = NodeScale(0);
		ConnectInfo(edge_count, 1) = NodeScale(2) - 2;
		edge_count++;
		ConnectInfo(edge_count, 0) = NodeScale(2) - NodeScale(0) - 3;
		ConnectInfo(edge_count, 1) = NodeScale(2) - 1;
		edge_count++;
		ConnectInfo(edge_count, 0) = NodeScale(2) - 4;
		ConnectInfo(edge_count, 1) = NodeScale(2);
		edge_count++;
		net.EdgeScale = edge_count;
		return true;
	}

	void Construct_MassVector(NET &net, double single_tether_mass, double bullet_mass) {
		MassVector.setZero();
		for (int i = 0; i < ConnectInfo.rows(); i++) {
			int node_1 = ConnectInfo(i, 0) - 1;
			int node_2 = ConnectInfo(i, 1) - 1;
			MassVector(node_1) += single_tether_mass / 2;
			MassVector(node_2) += single_tether_mass / 2;
		}
		// corner mass
		for (int corner_i = 0; corner_i < 4; corner_i++) {
			MassVector(NodeScale(1) * NodeScale(0) + corner_i) = bullet_mass;
		}
	}

	void AllocateNodeLocation(NET& net) {
		for (int i = 0; i < net.NodeScale(0); i++) {
			for (int j = 0; j < net.NodeScale(1); j++) {
				if (i * net.NodeScale(1) + j < net.NodeScale(2)) {
					net.Configuration[(i * net.NodeScale(1) + j)].Position << i * net.tether_initial_length, j * net.tether_initial_length, 0;
				}
			}
		}
		Vector3d temp1(-net.tether_initial_length * sqrt(2) / 2.0, -net.tether_initial_length * sqrt(2) / 2.0, 0);
		net.Configuration[net.NodeScale(2) - 4].Position = net.Configuration[0].Position + temp1;
		// corner mass 2
		Vector3d temp2(-net.tether_initial_length * sqrt(2) / 2.0, net.tether_initial_length * sqrt(2) / 2.0, 0);
		net.Configuration[net.NodeScale(2) - 3].Position = net.Configuration[net.NodeScale(1) - 1].Position + temp2;
		// corner mass 3
		Vector3d temp3(net.tether_initial_length * sqrt(2) / 2.0, -net.tether_initial_length * sqrt(2) / 2.0, 0);
		net.Configuration[net.NodeScale(2) - 2].Position = net.Configuration[net.NodeScale(2) - net.NodeScale(1) - 4].Position + temp3;
		// corner mass 4
		Vector3d temp4(net.tether_initial_length * sqrt(2) / 2.0, net.tether_initial_length * sqrt(2) / 2.0, 0);
		net.Configuration[net.NodeScale(2) - 1].Position = net.Configuration[net.NodeScale(2) - 1 - 4].Position + temp4;
		
		Vector3d offset(-1.75, -1.75, 0);

		for (int i = 0; i < net.NodeScale(2); i++) {
			net.Configuration[i].Position += offset;
			net.Configuration[i].Position *= 0.5;
		}
	}

	void NetSettings(NET &net) {
		for (int i = 0; i < net.NodeScale(2); i++) {
			net, Configuration[i].Velocity.setZero();
		}
		
		net.Configuration[net.NodeScale(2) - 1].Velocity << sqrt(2) / 2, sqrt(2) / 2, sqrt(3);
		net.Configuration[net.NodeScale(2) - 2].Velocity << sqrt(2) / 2, -sqrt(2) / 2, sqrt(3);
		net.Configuration[net.NodeScale(2) - 3].Velocity << -sqrt(2) / 2, sqrt(2) / 2, sqrt(3);
		net.Configuration[net.NodeScale(2) - 4].Velocity << -sqrt(2) / 2, -sqrt(2) / 2, sqrt(3);
		
	}
};
//////////////////////////////////////////////////////////////////////////////
// Construct class to represent a target (import from /obj initially)
//////////////////////////////////////////////////////////////////////////////
class TARGET {
public:
	class CENTERCONFIG {
	public:
		Vector3d Position;
		Vector3d Velocity;
		Vector3d EulerAngle;
	};

public:
	int num_face;
	int num_node;
	const double mass = 100;
	Matrix<double, Dynamic, 3> LocalPosition;
	CENTERCONFIG CenterConfiguration;
	Matrix<int, Dynamic, 3> MeshInfo;
	Matrix<double, Dynamic, 3> FaceNormal;

	TARGET(const char* OBJNAME, CENTERCONFIG init_center_configuration) {
		Matrix<double, Dynamic, 3> tempNormal;
		tempNormal.conservativeResize(0, 3);
		LocalPosition.conservativeResize(0, 3);
		MeshInfo.conservativeResize(0, 3);
		FaceNormal.conservativeResize(0, 3);
		ifstream infile(OBJNAME);
		if (!infile) {
			cerr << "Error opening file: " << OBJNAME << endl;
			exit(1);
		}

		num_face = 0;
		num_node = 0;
		CenterConfiguration = init_center_configuration;
		string line;
		while (getline(infile, line)) {
			if (line.substr(0, 2) == "v ") {
				// vertex position
				istringstream s(line.substr(2));
				Eigen::Vector3d v;
				s >> v(0);
				s >> v(1);
				s >> v(2);
				LocalPosition.conservativeResize(num_node + 1, 3);
				LocalPosition.row(num_node) = v;
				num_node++;
			}
			else if (line.substr(0, 2) == "vn") {
				// face normal
				istringstream s(line.substr(2));
				Eigen::Vector3d v;
				s >> v(0);
				s >> v(1);
				s >> v(2);
				tempNormal.conservativeResize(tempNormal.rows() + 1, 3);
				tempNormal.row(tempNormal.rows() - 1) = v;
			}
			else if (line.substr(0, 2) == "f ") {
				// face
				int normal_substr;
				Eigen::Vector3i v;
				istringstream s(line.substr(2, line.length()));
				int i = 0;
				while (i < 6) {
					if (i == 0) {
						s >> v(0);
					}
					else if (i == 2) {
						s >> v(1);
					}
					else if (i == 4) {
						s >> v(2);
					}
					else {
						string temp;
						s >> temp;
						istringstream ss(temp);
						char c1, c2;
						int n1, n2;
						ss >> c1;
						ss >> n1;
						ss >> c2;
						ss >> n2;
						normal_substr = n2;
						temp.empty();
					}
					i++;
				}
				FaceNormal.conservativeResize(num_face + 1, 3);
				FaceNormal.row(num_face) = tempNormal.row(normal_substr - 1);
				num_face++;
				MeshInfo.conservativeResize(MeshInfo.rows() + 1, 3);
				MeshInfo.row(MeshInfo.rows() - 1) = v;

			}
			else {
				continue;
			}
		}
	}
};

}