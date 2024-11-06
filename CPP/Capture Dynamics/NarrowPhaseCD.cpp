#include"NarrowPhaseCD.h"
#include<math.h>
#include<omp.h>
CONTACTPAIR NarrowPhaseCD(const NET &net, const TARGET &target, double TimeStep, double d_hat, Matrix<int, Dynamic, 2> PotentialCollisionPair) {
	CONTACTPAIR NarrowPhaseContactPair;
	NarrowPhaseContactPair.maxpenetration = 0;
	int count_collision_n2n = 0;
	int count_collision_n2f = 0;
	Matrix3d RotationMatrix;
	RotationMatrix = EulerRotationMatrix(target.CenterConfiguration.EulerAngle);
	int times = -1;
	while (times < PotentialCollisionPair.rows() - 1) {
		times ++;
		Vector2i tempPair = PotentialCollisionPair.row(times);
		if (tempPair(1) < 0 || tempPair(0) < 0) {

			double distance = Distance(net.Configuration[tempPair(0)].Position, RotationMatrix * target.FaceNormal.row(abs(tempPair(1)) - 1).transpose(), RotationMatrix * target.LocalPosition.row(target.MeshInfo(abs(tempPair(1)) - 1, 0) - 1).transpose() + target.CenterConfiguration.Position);

			if (distance <= net.NodeRadius && abs(distance) <= net.NodeRadius * 4) {

				Matrix3d FaceVertices;
				FaceVertices.row(0) = RotationMatrix * target.LocalPosition.row(target.MeshInfo(abs(tempPair(1)) - 1, 0) - 1).transpose() + target.CenterConfiguration.Position;
				FaceVertices.row(1) = RotationMatrix * target.LocalPosition.row(target.MeshInfo(abs(tempPair(1)) - 1, 1) - 1).transpose() + target.CenterConfiguration.Position;
				FaceVertices.row(2) = RotationMatrix * target.LocalPosition.row(target.MeshInfo(abs(tempPair(1)) - 1, 2) - 1).transpose() + target.CenterConfiguration.Position;
				if (!is_intriangle(FaceVertices, RotationMatrix * target.FaceNormal.row(abs(tempPair(1)) - 1).transpose(),
					net.Configuration[tempPair(0)].Position, distance)) {
					continue;
				}
				PAIRINFO tmp;
				tmp.Index << tempPair(0), abs(tempPair(1)) - 1;
				distance -= net.NodeRadius;
				if (abs(distance) > NarrowPhaseContactPair.maxpenetration) {
					NarrowPhaseContactPair.maxpenetration = distance;
				}
				tmp.distance = distance;
				;
				tmp.RelativeVelocity << target.CenterConfiguration.Velocity - net.Configuration[tempPair(0)].Velocity;
				if (NarrowPhaseContactPair.Node2Face.rows() == 0) {
					NarrowPhaseContactPair.Node2Face.conservativeResize(NarrowPhaseContactPair.Node2Face.rows() + 1, 1);
					NarrowPhaseContactPair.Node2Face.row(NarrowPhaseContactPair.Node2Face.rows() - 1) << tmp;
					count_collision_n2f++;
				}
				else {
					bool isrecorded = false;
#pragma omp parallel for
					for (int i = 0; i < NarrowPhaseContactPair.Node2Face.rows(); ++i) {
						if (tempPair(0) == NarrowPhaseContactPair.Node2Face(i, 0).Index(0)) {
							// calculate distance between face's vertices and net node
							// we simply choose the face having the smallest summed distance as the face to be collided
							if (distance < NarrowPhaseContactPair.Node2Face(i, 0).distance) {
								if (abs(distance) > NarrowPhaseContactPair.maxpenetration) {
									NarrowPhaseContactPair.maxpenetration = distance;
								}
								NarrowPhaseContactPair.Node2Face.row(i) << tmp;
								isrecorded = true;
								continue;
							}
							else {
								isrecorded = true;
							}
						}
					}
					if (!isrecorded) {
						NarrowPhaseContactPair.Node2Face.conservativeResize(NarrowPhaseContactPair.Node2Face.rows() + 1, 1);
						NarrowPhaseContactPair.Node2Face.row(NarrowPhaseContactPair.Node2Face.rows() - 1) << tmp;
						if (abs(distance) > NarrowPhaseContactPair.maxpenetration) {
							NarrowPhaseContactPair.maxpenetration = distance;
						}
						count_collision_n2f++;
						continue;
					}
				}
			}
			continue;
		}
		else {
			double distance = Distance(net.Configuration[tempPair(0)].Position, net.Configuration[tempPair(1)].Position);
			if (distance <= net.NodeRadius * 2) {
				PAIRINFO tmp;
				tmp.distance = distance;
				tmp.Index << tempPair(0), tempPair(1);
				tmp.RelativeVelocity << net.Configuration[tempPair(1)].Velocity - net.Configuration[tempPair(0)].Velocity;
				NarrowPhaseContactPair.Node2Node.conservativeResize(NarrowPhaseContactPair.Node2Face.rows() + 1, 1);
				NarrowPhaseContactPair.Node2Node.row(NarrowPhaseContactPair.Node2Node.rows() - 1) << tmp;
				count_collision_n2n++;
			}
		}

	}

	return NarrowPhaseContactPair;
}

double Distance(Vector3d vector1, Vector3d vector2) {
	return (vector1 - vector2).norm();
}
double Distance(Vector3d vector1, Vector3d normal, Vector3d facevertice) {
	Vector3d temp = vector1 - facevertice;
	double distance = (double) temp.dot(normal) / normal.norm();
	return distance;
}

bool is_intriangle(Matrix3d FaceVertices, Vector3d FaceNormal, Vector3d vector1, double distance) {
	Vector3d vector1_projeceted = vector1 - (0.001 + distance) * FaceNormal;
	bool isin = false;
	double a1 = calculate_area(FaceVertices.row(0) - vector1_projeceted.transpose(), FaceVertices.row(1) - vector1_projeceted.transpose());
	double a2 = calculate_area(FaceVertices.row(0) - vector1_projeceted.transpose(), FaceVertices.row(2) - vector1_projeceted.transpose());
	double a3 = calculate_area(FaceVertices.row(2) - vector1_projeceted.transpose(), FaceVertices.row(1) - vector1_projeceted.transpose());
	double sum_a = a1 + a2 + a3;
	double a = calculate_area(FaceVertices.row(1) - FaceVertices.row(0), FaceVertices.row(2) - FaceVertices.row(0));
	if (sum_a <= 1.001 * a) {
		isin = true;
	}
	return isin;
}
double calculate_area(Vector3d v1, Vector3d v2) {
	double area = (v1.cross(v2)).norm() * 0.50;
	return area;
}