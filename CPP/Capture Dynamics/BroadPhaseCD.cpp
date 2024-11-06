#include"BroadPhaseCD.h"
#include<omp.h>
bool is_contact(HASHGRID HashGrid, NET net, TARGET target, double TimeStep, double d_hat, Matrix<int, Dynamic, 2> &PotentialCollisionPair) {
	bool BroadPhaseContact = false;
	// Construct bounding volume for trajectories
	
	Vector3d Position_t0;
	Vector3d Position_t1;
	Vector3d Velocity_t0;
	int object_i = 0;
	while (object_i < net.NodeScale(2)) {
		Position_t0 = net.Configuration[object_i].Position;
		Velocity_t0 = net.Configuration[object_i].Velocity;
		Position_t1 = Position_t0 + TimeStep * Velocity_t0;
		BOX BV_node;
		BV_node << min(Position_t0(0), Position_t1(0)) - net.NodeRadius, min(Position_t0(1), Position_t1(1)) - net.NodeRadius, min(Position_t0(2), Position_t1(2)) - net.NodeRadius,
				   max(Position_t0(0), Position_t1(0)) + net.NodeRadius, max(Position_t0(1), Position_t1(1)) + net.NodeRadius, max(Position_t0(2), Position_t1(2)) + net.NodeRadius;
		BOX HashIndexInterval;
		////////////////// Give Interval of HashGrid //////////////////

		for (int i = 0; i < HashGrid.devide_num(0); ++i) {
			if (BV_node(0, 0) > HashGrid.HashGridInterval_X(i)) {
				HashIndexInterval(0, 0) = i + 1;
				continue;
			}
		}

		for (int i = HashGrid.devide_num(0); i > 0 ; --i) {
			if (BV_node(1, 0) < HashGrid.HashGridInterval_X(i)) {
				HashIndexInterval(1, 0) = i;
				continue;
			}
		}

		for (int i = 0; i < HashGrid.devide_num(1); ++i) {
			if (BV_node(0, 1) > HashGrid.HashGridInterval_Y(i)) {
				HashIndexInterval(0, 1) = i + 1;
				continue;
			}
		}

		for (int i = HashGrid.devide_num(1); i > 0; --i) {
			if (BV_node(1, 1) < HashGrid.HashGridInterval_Y(i)) {
				HashIndexInterval(1, 1) = i;
				continue;
			}
		}

		for (int i = 0; i < HashGrid.devide_num(2); ++i) {
			if (BV_node(0, 2) > HashGrid.HashGridInterval_Z(i)) {
				HashIndexInterval(0, 2) = i + 1;
				continue;
			}
		}

		for (int i = HashGrid.devide_num(2); i > 0; --i) {
			if (BV_node(1, 2) < HashGrid.HashGridInterval_Z(i)) {
				HashIndexInterval(1, 2) = i;
				continue;
			}
		}
		////////////////// End of Giving Interval of HashGrid //////////////////
		////////////////// Push index of location and object index into a priority queue

		for (int interval_i = 0; interval_i < (HashIndexInterval(1, 0) - HashIndexInterval(0, 0)) + 1; ++interval_i) {
			for (int interval_j = 0; interval_j < (HashIndexInterval(1, 1) - HashIndexInterval(0, 1)) + 1; ++interval_j) {
				for (int interval_k = 0; interval_k < (HashIndexInterval(1, 2) - HashIndexInterval(0, 2)) + 1; ++interval_k) {
					LOCATIONLIST IndexPair(HashIndexInterval(0, 0) + interval_i +
						(HashIndexInterval(0, 1) + interval_j - 1) * HashGrid.devide_num(0) + 
						(HashIndexInterval(0, 2) + interval_k - 1) * HashGrid.devide_num(0) * HashGrid.devide_num(1), object_i);
					HashGrid.BoundingBoxLocation.push(IndexPair);
				}
			}
		}
		object_i++;
	}

	/////////////////////////////////////////////////////////////////////////////
	//////////Construct face bounding volume/////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////

	int face_i = 0;
	while (face_i < target.num_face) {
		BOX BV_target;
		MatrixXd temp;
		MatrixXd TargetNodePosition;
		TargetNodePosition.conservativeResizeLike(target.LocalPosition);
		temp.conservativeResizeLike(target.LocalPosition);

		for (int row_i = 0; row_i < target.LocalPosition.rows(); ++row_i) {
			TargetNodePosition.row(row_i) = EulerRotationMatrix(target.CenterConfiguration.EulerAngle) * target.LocalPosition.row(row_i).transpose() + target.CenterConfiguration.Position;
		}
		
		Matrix3d FaceVerticesPosition;
		FaceVerticesPosition.row(0) = TargetNodePosition.row(target.MeshInfo(face_i, 0) - 1);
		FaceVerticesPosition.row(1) = TargetNodePosition.row(target.MeshInfo(face_i, 1) - 1);
		FaceVerticesPosition.row(2) = TargetNodePosition.row(target.MeshInfo(face_i, 2) - 1);
		BV_target << FaceVerticesPosition.col(0).minCoeff() - d_hat, FaceVerticesPosition.col(1).minCoeff() - d_hat, FaceVerticesPosition.col(2).minCoeff() - d_hat,
			FaceVerticesPosition.col(0).maxCoeff() + d_hat, FaceVerticesPosition.col(1).maxCoeff() + d_hat, FaceVerticesPosition.col(2).maxCoeff() + d_hat;

		BOX HashIndexInterval;
		////////////////// Give Interval of HashGrid //////////////////
		for (int i = 0; i < HashGrid.devide_num(0); ++i) {
			if (BV_target(0, 0) > HashGrid.HashGridInterval_X(i)) {
				HashIndexInterval(0, 0) = i + 1;
				continue;
			}
		}
		for (int i = HashGrid.devide_num(0); i > 0; --i) {
			if (BV_target(1, 0) < HashGrid.HashGridInterval_X(i)) {
				HashIndexInterval(1, 0) = i;
				continue;
			}
		}
		for (int i = 0; i < HashGrid.devide_num(1); ++i) {
			if (BV_target(0, 1) > HashGrid.HashGridInterval_Y(i)) {
				HashIndexInterval(0, 1) = i + 1;
				continue;
			}
		}

		for (int i = HashGrid.devide_num(1); i > 0; --i) {
			if (BV_target(1, 1) < HashGrid.HashGridInterval_Y(i)) {
				HashIndexInterval(1, 1) = i;
				continue;
			}
		}


		for (int i = 0; i < HashGrid.devide_num(2); ++i) {
			if (BV_target(0, 2) > HashGrid.HashGridInterval_Z(i)) {
				HashIndexInterval(0, 2) = i + 1;
				continue;
			}
		}
		if (abs(HashIndexInterval(0, 2)) > 100) {
			int b = 1;
		}


		for (int i = HashGrid.devide_num(2); i > 0; --i) {
			if (BV_target(1, 2) < HashGrid.HashGridInterval_Z(i)) {
				HashIndexInterval(1, 2) = i;
				continue;
			}
		}
		////////////////// End of Giving Interval of HashGrid //////////////////
		for (int interval_i = 0; interval_i < (HashIndexInterval(1, 0) - HashIndexInterval(0, 0)) + 1; ++interval_i) {
			for (int interval_j = 0; interval_j < (HashIndexInterval(1, 1) - HashIndexInterval(0, 1)) + 1; ++interval_j) {
				for (int interval_k = 0; interval_k < (HashIndexInterval(1, 2) - HashIndexInterval(0, 2)) + 1; ++interval_k) {
					LOCATIONLIST IndexPair(HashIndexInterval(0, 0) + interval_i +
						(HashIndexInterval(0, 1) + interval_j - 1) * HashGrid.devide_num(0) +
						(HashIndexInterval(0, 2) + interval_k - 1) * HashGrid.devide_num(0) * HashGrid.devide_num(1), -face_i-1);

					HashGrid.BoundingBoxLocation.push(IndexPair);
				}
			}
		}
		face_i++;
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////// GO BROAD-PHASE GO ////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int pre_locationIndex = HashGrid.BoundingBoxLocation.top().x.first;
	int pre_objectIndex = HashGrid.BoundingBoxLocation.top().x.second;
	HashGrid.BoundingBoxLocation.pop();
	int now_locationIndex = HashGrid.BoundingBoxLocation.top().x.first;
	int now_objectIndex = HashGrid.BoundingBoxLocation.top().x.second;

	while (!HashGrid.BoundingBoxLocation.empty()) {

		MatrixXd TEMP = MatrixXd::Zero(1, 1);
		TEMP(0) = pre_objectIndex;
		if (now_locationIndex == pre_locationIndex) {
			do {
				TEMP.conservativeResize(TEMP.size() + 1, 1);
				TEMP(TEMP.size()-1) = now_objectIndex;
				HashGrid.BoundingBoxLocation.pop();
				if (HashGrid.BoundingBoxLocation.empty()) {
					break;
				}
				now_locationIndex = HashGrid.BoundingBoxLocation.top().x.first;
				now_objectIndex = HashGrid.BoundingBoxLocation.top().x.second;
			} while (now_locationIndex == pre_locationIndex);


			for (int i = 0; i < TEMP.size()-1; ++i) {
				for (int j = i + 1; j < TEMP.size(); ++j) {
					if (TEMP(i) < 0 && TEMP(j) < 0) {
						continue;
					}
					else {
						//////// Check Repeat Pair ////////
						bool isrecord = false;
						COLLISIONPAIRLIST pairname(TEMP(i), TEMP(j));
						if (pairname.x.first < pairname.x.second) {
							int tmp = pairname.x.first;
							pairname.x.first = pairname.x.second;
							pairname.x.second = tmp;
						}
						for (int i = 0; i < PotentialCollisionPair.rows(); ++i) {
							if (pairname.x.first == PotentialCollisionPair(i, 0) && pairname.x.second == PotentialCollisionPair(i, 1)) {
								isrecord = true;
							}
						}
						if (!isrecord) {
							PotentialCollisionPair.conservativeResize(PotentialCollisionPair.rows() + 1, 2);
							PotentialCollisionPair.row(PotentialCollisionPair.rows() - 1) << pairname.x.first, pairname.x.second;
							if (pairname.x.first == pairname.x.second) {
								cout << " ERROR! NODE COLLIDES TO ITSELF!" << endl;
							}
						}
					}

				}
			}
		}
		else {
			pre_locationIndex = now_locationIndex;
			pre_objectIndex = now_objectIndex;
			HashGrid.BoundingBoxLocation.pop();
			if (HashGrid.BoundingBoxLocation.empty()) {
				break;
			}
			now_locationIndex = HashGrid.BoundingBoxLocation.top().x.first;
			now_objectIndex = HashGrid.BoundingBoxLocation.top().x.second;
		}
	}
	
	if (PotentialCollisionPair.rows() > 0) {
		BroadPhaseContact = true;
	}
	

	return BroadPhaseContact;
}

Matrix3d EulerRotationMatrix(Vector3d EulerAngle) {
	Matrix3d RotationMatrix;
	RotationMatrix.setZero();
	RotationMatrix = AngleAxisd(EulerAngle(2), Vector3d::UnitZ()) *
		AngleAxisd(EulerAngle(1), Vector3d::UnitY()) *
		AngleAxisd(EulerAngle(0), Vector3d::UnitX());
	return RotationMatrix;
}


MatrixXd RodriguesMatrix(double Theta, Vector3d AxisDirection) {
	Matrix3d R;
	R.Identity();
	R += SkewMatrix(AxisDirection) * sin(Theta) + SkewMatrix(AxisDirection) * SkewMatrix(AxisDirection).transpose() * (1 - cos(Theta));
	return R;
}

MatrixXd SkewMatrix(Vector3d AxisDirection){
	Matrix3d S;
	S.setZero();
	S(0, 1) = -AxisDirection(2);
	S(0, 2) = AxisDirection(1);
	S(1, 0) = AxisDirection(2);
	S(1, 2) = -AxisDirection(0);
	S(2, 0) = -AxisDirection(1);
	S(2, 1) = AxisDirection(0);
	return S;
}