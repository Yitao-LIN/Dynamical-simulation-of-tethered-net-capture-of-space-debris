#include"initial_condition.h"
using namespace std;
using namespace Eigen;
NodeState Corner_Z_velocity_mode(NodeState NodeStat, double velocity, int n_row, int n_col) {
	int n_node = n_row * n_col;

	NodeStat.velocity.col(2).setOnes();

	return NodeStat;
}