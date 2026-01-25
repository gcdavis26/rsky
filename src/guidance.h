#pragma once
#include <Eigen/Dense>

class Guidance
{
private:
	Eigen::Matrix<double, 12, 1> x0;
	int phase; //1 = takeoff. 2 = search. 3 = descend. 4 = Hover, drop payload. 5 = retreat to some point. 6 = land. 
	int passes;
	double stripe_x;
	Eigen::Vector2d xbounds; 
	Eigen::Vector2d ybounds; 
	double cruise;
	double takeoff_height;
	bool found;
	double delta_x;
	int dir; 
	bool at_end;
	double fov;
	double K;
	Eigen::VectorXd lawnmower_stripes;

public:
	Guidance::Guidance(Eigen::Matrix<double, 12, 1> x, Eigen::Vector2d x_bounds, Eigen::Vector2d y_bounds, int numpasses, double cruise_speed, double takeoff, double gain);
	Eigen::Matrix<double, 10, 1> Guidance::getTarget(Eigen::Matrix<double, 12, 1> x);
};

