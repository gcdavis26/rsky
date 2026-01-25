#include "guidance.h"
#include <Eigen/Dense>
#include "math_utils.h"
#include <cmath>
#include <iostream>

Guidance::Guidance(Eigen::Matrix<double,12,1> x, Eigen::Vector2d x_bounds, Eigen::Vector2d y_bounds, int numpasses, double cruise_speed, double takeoff, double gain)
{
	x0 = x;
	phase = 1;
	found = false;
	at_end = false;
	xbounds = x_bounds;
	ybounds = y_bounds;
	passes = numpasses;
	takeoff_height = takeoff;
	cruise = cruise_speed;
	fov = 6 * .3048; //6 foot
	lawnmower_stripes.resize(passes);
	lawnmower_stripes.setZero();
	delta_x = (xbounds(1) - xbounds(0) - fov) / (passes - 1);
	for (int i = 0; i < passes; i++)
	{
		lawnmower_stripes(i) = xbounds(0) + fov / 2 + i * delta_x;
	}
	K = gain;
	dir = 1;
	stripe_x = 0;

}
Eigen::Matrix<double, 10, 1> Guidance::getTarget(Eigen::Matrix<double, 12, 1> x)
{
	Eigen::Matrix<double, 10, 1> commands;
	double psides;
	Eigen::Vector3d omegades;
	Eigen::Vector3d pdes;
	Eigen::Vector3d vdes;
	switch (phase)
	{
	case 1:
	{
		Eigen::Vector2d center;
		center << xbounds.mean(), ybounds.mean();
		Eigen::Vector2d xy_des = (center - x0.block(6, 0, 2, 1)) / (center - x0.block(6, 0, 2, 1)).norm() * fov / 2;

		psides = x(2);
		omegades = Eigen::Vector3d::Zero();
		pdes << xy_des(0), xy_des(1), -takeoff_height;
		vdes = Eigen::Vector3d::Zero();
		double res = std::abs(pdes(2) - x(8));
		if (res < .02)
		{
			phase += 1;
			std::cout << "Takeoff completed, beginning search" << std::endl;	
			Eigen::Index maxIndex;
			((xbounds.array() - x(6)).cwiseAbs()).maxCoeff(&maxIndex);
			if (maxIndex == 0)
			{
				dir = 1; //going left to right
			}
			else
			{
				dir = -1; //going right to left
			}

		}
		break;
	}
	case 2:
	{
		Eigen::Index minIndex;
		((lawnmower_stripes.array() - x(6)).cwiseAbs()).minCoeff(&minIndex);
		stripe_x = lawnmower_stripes(minIndex);
		std::cout << stripe_x << std::endl;
		double vx = K * (stripe_x - x(6));
		double vy = cruise;
		if (!at_end && (x(7) > ybounds(1) || x(7) < ybounds(0))) //!at_end prevents this from triggering constantly outside of bounds
		{
			at_end = true;
			vy *= -1;
			if ((minIndex + dir) >= passes || (minIndex + dir) < 0)
			{
				dir = -dir;
			}
			stripe_x = lawnmower_stripes(minIndex + dir);
		}
		else if (x(7) <= ybounds(1) && x(7) >= ybounds(0))
			at_end = false;
		psides = x(2);
		omegades = Eigen::Vector3d::Zero();
		pdes << x(6), x(7), -takeoff_height;
		vdes << vx, vy, 0;
		break;
	}
	case 3:
	{
		std::exit(0);
		break;
	}
	case 4:
	{
		break;
	}
	}
	commands << psides, omegades, pdes, vdes;
	//returns psides, omegades, pdes, vdes
	return commands;
	
}