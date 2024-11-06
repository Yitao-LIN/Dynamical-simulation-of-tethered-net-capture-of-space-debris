#pragma once
#include<iostream>
#include<Eigen/Dense>
#include"Force.h"
#include<iomanip>
#include"BroadPhaseCD.h"
#include"NarrowPhaseCD.h"
#include"Object.h"
using namespace std;
using namespace Eigen;
using namespace OBJ;
MatrixXd Calculate_force(NET net, TARGET target, CONTACTPAIR& NarrowPhaseContactPair);
void ContactForceAndMoment(PAIRINFO PairInfo, NET net, TARGET target, double k, double d, double friction_factor, Vector3d& Fn, Vector3d& Ft, Vector3d& M);