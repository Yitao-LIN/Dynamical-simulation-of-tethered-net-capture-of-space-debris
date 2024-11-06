#pragma once
#include<iostream>
#include<Eigen/Eigen>
#include"Force.h"

using namespace std;
using namespace Eigen;

void Runge_Kutta(NET& net, double TimeStep, double d_hat, ofstream& netacc, int fcout_calculator);
//void clear_priority_queue(priority_queue<COLLISIONPAIRLIST>& q);