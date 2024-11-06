#include <iostream>
#include<fstream>

using namespace std;

class Ball_1D {
public:
	double position;
	double velosity;
	double acceleration;
	double m;
	double t;
};

Ball_1D Runge_Kutta(Ball_1D pre_state, double step, double stiffness, double c);

int main() {
	Ball_1D ball;
	ball.position = 1.0;
	ball.velosity = 0.0;
	ball.acceleration = 0.0;
	ball.m = 1.0;
	ball.t = 0.0;
	double c = 1;
	double stiffness = 500.0;
	
	// Runge Kutta Method
	double total_time = 2.0;
	double step_time = 0.01;

	ofstream txtfile, txtfile_1, txtfile_2;
	txtfile.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/Runge_Kutta/output/position.txt");
	txtfile_1.open("T:/#Rifle在学习/# Graduation Thesis/进度/coding/Runge_Kutta/output/velosity.txt");

	while (ball.t <= 2.0) {
		txtfile << ball.position << endl;
		txtfile_1 << ball.velosity << endl;
		ball = Runge_Kutta(ball, step_time, stiffness, c);

	}

	return 0;
}

Ball_1D Runge_Kutta(Ball_1D pre_state, double step, double stiffness, double c) {
	double F = -pre_state.m * 9.8;
	double spring_length_0 = 1.0;

	Ball_1D new_state = pre_state;
	double f_1[2];
	f_1[0] = step * pre_state.velosity;
	f_1[1] = step * (F - stiffness * (pre_state.position - spring_length_0) - c * (pre_state.velosity));
	double f_2[2];
	f_2[0] = step * (pre_state.velosity + f_1[1] / 2);
	f_2[1] = step * (F - stiffness * ((pre_state.position + f_1[0] / 2) - spring_length_0) - c * (pre_state.velosity + f_1[1] / 2));
	double f_3[2];
	f_3[0] = step * (pre_state.velosity + f_2[1] / 2);
	f_3[1] = step * (F - stiffness * ((pre_state.position + f_2[0] / 2) - spring_length_0) - c * (pre_state.velosity + f_2[1] / 2));
	double f_4[2];
	f_4[0] = step * (pre_state.velosity + f_3[1]);
	f_4[1] = step * (F - stiffness * ((pre_state.position + f_3[0]) - spring_length_0) - c * (pre_state.velosity + f_3[1]));

	new_state.position = pre_state.position + (f_1[0] + 2 * f_2[0] + 2 * f_3[0] + f_4[0]) / 6;
	new_state.velosity = pre_state.velosity + (f_1[1] + 2 * f_2[1] + 2 * f_3[1] + f_4[1]) / 6;
	new_state.t = pre_state.t + step;
	return new_state;
}