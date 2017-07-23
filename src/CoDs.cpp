
/**
 *  @file    CoDs.cpp
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


#include "CoDs.h"


/**
 *   @brief  Initializing the CoDs package.
 *
 *   @param  Dimen_state is the dimension of the state.
 *   @param  delta_dx is the desired velocity of the robot at the impact point.
 *   @param  F_d is the desired force of the robot at the contact phase.
 *   @param  defined_surface is a boolean. If it is true, the package uses a planner surface defined in  the Gamma class. Otherwise it uses the user defined Gamma function.
 *   @return void
 */
void CoDs::initialize(int Dimen_state,double delta_dx,double F_d, bool defined_surface)
{

	Dimen_state_=Dimen_state;
	Surface_=defined_surface;
}

/**
 *   @brief Setting the value of Gamma and the normal to the surface
 *
 *   @param  Gamma is the surface; i.e. 1<=Gamma is the free motion phase 0<Gamma<1 the transition phase and Gamma=<0 the contact phase.
 *   @param  Normal is the current normal vector to the surface.
 *   @param  q2 is the tangential vector to N;
 *   @param  q3 is the tangential vector to N and q3;
 *   @return void
 */

void CoDs::Set_Gamma(double Gamma,VectorXd Normal,VectorXd q2,VectorXd q3)
{

	if ((Normal.cols()==Dimen_state_)&&(q2.cols()==Dimen_state_)&&(q3.cols()==Dimen_state_)&&((N_.transpose()*q2_+N_.transpose()*q3_+q2.transpose()*q3_)(1,1)<0.0001))
	{
	}
	else
	{
		cout<<"Something is wrong in Set_Gamma"<<endl;
		cout<<"Dimension of states is: "<<Dimen_state_<<endl;
		cout<<"N_ "<<N_<<endl;
		cout<<"q2_ "<<q2_<<endl;
		cout<<"q3_ "<<q3_<<endl;
		Error();
	}



	Gamma_Value_=Gamma;
	N_=Normal;
	q2_=q2;
	q3_=q3;
}

/**
 *   @brief Setting the state of the system and the original dynamic
 *
 *   @param  State is the state of the system.
 *   @param  Original_Dynamic is the original dynamic.
 *   @return void
 */

void CoDs::Set_State(VectorXd State,VectorXd Original_Dynamic)
{

	if ((State.cols()==Dimen_state_)&&(Original_Dynamic.cols()==Dimen_state_))
	{
	}
	else
	{
		cout<<"Something is wrong in Set_State"<<endl;
		cout<<"Dimension of states is: "<<Dimen_state_<<endl;
		cout<<"State "<<State<<endl;
		cout<<"Original_Dynamic "<<Original_Dynamic<<endl;
		Error();
	}

	X_=State;
	F_=Original_Dynamic;
}


//Gamma,f_x,DX,X,X_C,Q,Qinv,Contact




/**
 *   @brief  If something is wrong, it is activated.
 *
 *   @return void
 */
void CoDs::Error()
{


	while(ros::ok())
	{

	}

}







































/**
 *   @brief  If something is wrong, it is activated.
 *
 *   @return void
 */
void Gamma::Error()
{


	while(ros::ok())
	{

	}

}
