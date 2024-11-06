#pragma once
#include<iostream>
#include<Eigen/Eigen>
#include"Force.h"
#include"BroadPhaseCD.h"
#include"NarrowPhaseCD.h"

using namespace std;
using namespace Eigen;

void Runge_Kutta(NET& net, TARGET& target, double TimeStep, double d_hat, HASHGRID& HashGrid);
//void clear_priority_queue(priority_queue<COLLISIONPAIRLIST>& q);