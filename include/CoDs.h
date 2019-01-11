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
#include <math.h>
#include "ros/ros.h"

using namespace std;
using namespace Eigen;

const double epsilon=10e-3;

enum ENUM_Phase{Phase_Free_motion=0,Phase_Transition,Phase_Contact,Phase_Everything_is_done};

class CoDs
{
public:
	void initialize(int Dimen_state,double delta_dx,double F_d,double Gammma_free_motion, bool define_desired_contact_point, bool define_desired_leaving_point);
	void Set_Gamma(double Gamma,VectorXd Normal,VectorXd q2,VectorXd q3,VectorXd Point_on_surface);
	void Set_State(VectorXd State,VectorXd DState,VectorXd DState_real,VectorXd Original_Dynamic);
	void Set_Contact_point(VectorXd Contact_point);
	VectorXd Set_Leaving_point(VectorXd Leaving_point,VectorXd X_Target);
	void Set_Mass_Matrix(MatrixXd M);


	MatrixXd Calculate_Modulation();

	VectorXd Get_Normal_Velocity();

	double Get_Modulated_Surface();

	VectorXd Get_Modulated_Target();

	bool	Is_the_task_being_done();
	ENUM_Phase	Which_phase_is_it();

private:

	void Error();
	inline	bool everythingisreceived();
	inline	void everyfalse();

//	bool Surface_;
	bool Leaving_point_;
	bool Contact_point_;
	bool State_of_system_is_set_;
	bool State_of_surface_is_set_;
	bool State_of_contact_is_set_;
	bool State_of_leaving_is_set_;

	bool Motion_Phases_[1+1+1+1];// Phase 0=transition phase 1= Contact phase 2=leaving phase 3 everything is done

	ENUM_Phase	Phase_of_the_motion_;

	int Dimen_state_;


	double Gamma_Value_;
	double Gamma_Modulated_Value_;
	double Gammma_Threshold_;

	double delta_dx_;
	double F_d_;

	double	handle_N_;

	double 	nu_;


	double Omega_;

	double handle_double_;

	VectorXd qF_;

	VectorXd N_;
	VectorXd q2_;
	VectorXd q3_;
	VectorXd Point_;

	VectorXd Desired_Contact_point_;
	VectorXd Desired_Leaving_point_;

	VectorXd X_;
	VectorXd DX_;
	VectorXd DXState_real_;
	VectorXd F_;

	VectorXd X_Target_Modulated_;

	MatrixXd Q_;
	MatrixXd Q_inv_;
	MatrixXd Lambda_;
	MatrixXd Lambda_Bold_;
	MatrixXd M_;


	MatrixXd InvMass_;


	VectorXd Normal_velocity_robot_;
	VectorXd Normal_velocity_robot_real_;




};

/*

class Gamma
{
public:



private:

	void Error();

};
*/



