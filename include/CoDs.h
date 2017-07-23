/**
 *  @file    CoDs.h
 *  @author  Sina Mirrazavi (sina.mirrazavi@epfl.ch)
 *  @date    2017
 *  @version 1.0
 *
 *  @brief Smooth transitions  from non/contact to contact environment
 *
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"

using namespace std;
using namespace Eigen;


class CoDs
{
public:
	void initialize(int Dimen_state,double delta_dx,double F_d,bool defined_surface);
	void Set_Gamma(double Gamma,VectorXd Normal,VectorXd q2,VectorXd q3);
	void Set_State(VectorXd State,VectorXd Original_Dynamic);


private:

	void Error();

	bool Surface_;


	int Dimen_state_;


	double Gamma_Value_;


	VectorXd N_;
	VectorXd q2_;
	VectorXd q3_;
	VectorXd X_;
	VectorXd F_;


};


class Gamma
{
public:



private:

	void Error();

};



