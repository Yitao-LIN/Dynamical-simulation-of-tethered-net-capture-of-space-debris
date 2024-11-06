#pragma once
#include<iostream>
#include<Eigen/Dense>
#include"Force.h"
#include<iomanip>
#include"Object.h"
using namespace std;
using namespace Eigen;
using namespace OBJ;
MatrixXd Calculate_force(NET net);
