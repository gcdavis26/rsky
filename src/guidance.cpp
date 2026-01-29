#include "guidance.h"
#include <Eigen/Dense>
#include "math_utils.h"
#include <cmath>
#include <iostream>

Guidance::Guidance(Eigen::Matrix<double,12,1> x, Eigen::Vector2d n_bounds, Eigen::Vector2d e_bounds, int numpasses, double cruise_speed, double takeoff, double gain)
{
	x0 = x;
	phase = 1;
	found = false;
	at_end = false;
	nbounds = n_bounds;
	ebounds = e_bounds;
	passes = numpasses;
	takeoff_height = takeoff;
	cruise = cruise_speed;
	fov = 6 * .3048; //6 foot
	lawnmower_stripes.resize(passes);
	lawnmower_stripes.setZero();
	delta_e = (ebounds(1) - ebounds(0) - fov/2) / (passes - 1); //distance between stripes
	for (int i = 0; i < passes; i++)
	{
		lawnmower_stripes(i) = ebounds(0) + fov / 4 + i * delta_e; 
	}
	std::cout << "All planned stripes: " << std::endl << lawnmower_stripes << std::endl;
	((lawnmower_stripes.array() - x(7)).cwiseAbs()).minCoeff(&stripe_index); //initializing stripe 
	K = gain;
	edir = 1;
	ndir = 1;

	//north is to the right, east is up. Stripes will be along east. 

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
	case 1: //takeoff
	{
		Eigen::Vector2d center;
		center <<nbounds.mean(), ebounds.mean();
		Eigen::Vector2d ne_des = (center - x0.block(6, 0, 2, 1)) / (center - x0.block(6, 0, 2, 1)).norm() * fov;

		psides = x(2);
		omegades = Eigen::Vector3d::Zero();
		pdes << ne_des(0), ne_des(1), -takeoff_height;
		vdes = Eigen::Vector3d::Zero();
		double res = std::abs(pdes(2) - x(8));
		if (res < .02)
		{

			//done with takeoff, this is now initializing the search pattern
			phase += 1;
			std::cout << "Takeoff completed, beginning search" << std::endl;	
			Eigen::Index minIndex;
			((ebounds.array() - x(7)).cwiseAbs()).minCoeff(&minIndex); //finding nearest stripe
			if (minIndex == 0)
			{
				edir = 1; //going left to right
			}
			else
			{
				edir = -1; //going right to left
			}

			((nbounds.array() - x(6)).cwiseAbs()).minCoeff(&minIndex);
			if (minIndex == 0)
			{
				ndir = 1; //going up
			}
			else
			{
				ndir = -1; //going down
			}

			((lawnmower_stripes.array() - x(7)).cwiseAbs()).minCoeff(&stripe_index);
		}
		break;
	}
	case 2: //searching
	{	
		double target_e = lawnmower_stripes(stripe_index);
		double buffer = fov/4;
		double ve;
		double vn;
		if (x(6) > nbounds(1)-buffer || x(6) < nbounds(0)+buffer) //!at_end prevents this from triggering constantly outside of bounds
		{
			if (!at_end)
			{
				at_end = true;
				ndir *= -1; //switch direction along stripe
				if ((stripe_index + edir) >= passes || (stripe_index + edir) < 0) //if we're at the end of the lawnmower, restart by going the other way
				{
					edir = -edir;
				}
				std::cout << "OLD stripe:" << std::endl << lawnmower_stripes(stripe_index) << std::endl;
				stripe_index += edir; //move to next stripe
				target_e = lawnmower_stripes(stripe_index);
				std::cout << "NEW stripe:" << std::endl << lawnmower_stripes(stripe_index) << std::endl;
				std::cout << "POS" << std::endl << x.block(6, 0, 2, 1) << std::endl << std::endl;
				ve = 0;
			}
			else if (x(9)*ndir < 0)
			{
				ve = 0;
			}
			else
			{
				ve = K * (target_e - x(7));
			}
		}
		else
		{//when we go back in bounds
			ve = K * (target_e - x(7));
			at_end = false;
		}
		ve = saturate(ve,cruise);
		vn = ndir * cruise;
		pdes << x(6), x(7), -takeoff_height; //not commanding n and e: Error is 0, velocity field should keep these in touch
		
		//extra nudge to push in bounds if we're going outside of the true bounds, as this can be catastrophic.
		if (x(6) > nbounds(1))
		{
			pdes(0) = x(6) - fov / 8;
		}
		else if (x(6) < nbounds(0))
		{
			pdes(0) = x(6) + fov / 8;
		}
		if (x(7) > ebounds(1))
		{
			pdes(1) = x(7) - fov / 8;
		}
		else if (x(7) < ebounds(0))
		{
			pdes(1) = x(7) + fov / 8;
		}
		
		psides = x(2); 
		omegades = Eigen::Vector3d::Zero();
		vdes << vn, ve, 0;
		break;
	}
	case 3: //logic from fire detection would go into these cases
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