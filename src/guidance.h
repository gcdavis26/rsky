#pragma once
#include <Eigen/Dense>
#include "math_utils.h"
class Guidance
{
private:
	Eigen::Matrix<double, 12, 1> x0;
	int phase; //1 = takeoff. 2 = search. 3 = descend. 4 = Hover, drop payload. 5 = retreat to some point. 6 = land. 
	int passes;
	Eigen::Index stripe_index;
	Eigen::Vector2d nbounds; 
	Eigen::Vector2d ebounds; 
	double cruise;
	double takeoff_height;
	bool found;
	double delta_e;
	int edir; 
	int ndir;
	bool at_end;
	double fov;
	double buffer;
	double Kp;
	double Kd;
	Eigen::VectorXd lawnmower_stripes;
public:
	Guidance::Guidance(Vector12d x, Vector2d n_bounds, Vector2d e_bounds, int numpasses, double cruise_speed, double takeoff, double pgain, double vgain);
	Vector10d Guidance::getTarget(Vector12d x);
};

