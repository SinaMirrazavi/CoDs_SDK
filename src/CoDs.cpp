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
 *   @param  Dimen_state is the dimension of the state. It should be 3!
 *   @param  delta_dx is the desired velocity of the robot at the impact point.
 *   @param  Gammma_free_motion is the threshold which defines boundary between the free motion and the transition phases.
 *   @param  define_desired_contact_point is a boolean. If it is true, the robot hits the surfaces at the specified point.
 *   @param  define_desired_leaving_point is a boolean. If it is true, the robot leaves the surfaces at the specified point.
 *   @return void
 */
//   @param  defined_surface is a boolean. If it is true, the package uses a planner surface defined in  the Gamma class. Otherwise it uses the user defined Gamma function.
void CoDs::initialize(int Dimen_state,double delta_dx,double Gammma_free_motion, bool define_desired_contact_point, bool define_desired_leaving_point)
{

	if (Dimen_state==3)
	{

	}
	else
	{
		cout<<"The dimension of the system should be 3; i.e. x y z!"<<endl;
		cout<<"Dimen_state "<<Dimen_state<<endl;
		Error();
	}
	if (0<delta_dx)
	{
		cout<<"delta_dx should be either zero or negative! "<<delta_dx<<endl;
		Error();
	}
	Dimen_state_=Dimen_state;
	//	Surface_=defined_surface;
	Leaving_point_=define_desired_leaving_point;
	Contact_point_=define_desired_contact_point;
	Gammma_Threshold_=Gammma_free_motion;

	delta_dx_=delta_dx;



	N_.resize(Dimen_state_);	q2_.resize(Dimen_state_);	q3_.resize(Dimen_state_);
	X_.resize(Dimen_state_);	DX_.resize(Dimen_state_);	F_.resize(Dimen_state_);
	DXState_real_.resize(Dimen_state_);
	X_Target_Modulated_.resize(Dimen_state_);

	Gamma_Value_=0;



	Q_.resize(Dimen_state_,Dimen_state_); 			Q_.setZero();
	Q_inv_.resize(Dimen_state_,Dimen_state_); 		Q_inv_.setZero();
	Lambda_.resize(Dimen_state_,Dimen_state_); 		Lambda_.setZero();
	Lambda_Bold_.resize(Dimen_state_,Dimen_state_); Lambda_Bold_.setZero();
	M_.resize(Dimen_state_,Dimen_state_); 			M_.setZero();


	qF_.resize(Dimen_state_);						qF_.setZero();


	Normal_velocity_robot_.resize(Dimen_state_); Normal_velocity_robot_.setZero();
	Normal_velocity_robot_real_.resize(Dimen_state_); Normal_velocity_robot_real_.setZero();

	Motion_Phases_[0]=false;
	Motion_Phases_[1]=false;
	Motion_Phases_[2]=false;
	Motion_Phases_[3]=false;


	Omega_=5/Gammma_Threshold_;
	nu_=-0.1*delta_dx_;

	Phase_of_the_motion_=Phase_Free_motion;
	everyfalse();

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

	//if ((Normal.rows()==Dimen_state_)&&(q2.rows()==Dimen_state_)&&(q3.rows()==Dimen_state_)&&((N_.transpose()*q2_+N_.transpose()*q3_+q2.transpose()*q3_)(0,0)<0.0001))
	if ((Normal.rows()==Dimen_state_)&&(q2.rows()==Dimen_state_)&&(q3.rows()==Dimen_state_)&&((Normal.transpose()*q2+Normal.transpose()*q3+q2.transpose()*q3)(0,0)<0.1))
	{
	}
	else
	{
		cout<<"Something is wrong in Set_Gamma"<<endl;
		cout<<"Dimension of states is: "<<Dimen_state_<<endl;
		cout<<"Normal "<<Normal.cols()<<endl;cout<<Normal<<endl;
		cout<<"q2 "<<q2.cols()<<endl;cout<<q2<<endl;
		cout<<"q3 "<<q3.cols()<<endl;cout<<q3<<endl;
		cout<<"(q2.transpose()*q3_)(0,0) "<<(q2.transpose()*q3)(0,0)<<endl;
		cout<<"(N_.transpose()*q3_)(0,0) "<<(Normal.transpose()*q3)(0,0)<<endl;
		cout<<"(N_.transpose()*q2_)(0,0) "<<(Normal.transpose()*q2)(0,0)<<endl;
		Error();
	}



	Gamma_Value_=Gamma;
	N_=Normal;
	q2_=q2;
	q3_=q3;

	Q_.col(0)=N_;	Q_.col(1)=q2_;	Q_.col(2)=q3_;

	Q_inv_=Q_.inverse();
	State_of_surface_is_set_=true;
}

/**
 *   @brief Setting the state of the system and the original dynamic
 *
 *   @param  State is the state of the system.
 *   @param  Original_Dynamic is the nominal dynamic.
 *   @return void
 */
void CoDs::Set_State(VectorXd State,VectorXd DState,VectorXd DState_real,VectorXd Original_Dynamic)
{

	if ((State.rows()==Dimen_state_)&&(Original_Dynamic.rows()==Dimen_state_)&&(DState.rows()==Dimen_state_)&&(DState_real.rows()==Dimen_state_))
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
	DX_=DState;
	DXState_real_=DState_real;
	F_=Original_Dynamic;
	State_of_system_is_set_=true;
}


/**
 *   @brief Calculate the modulation function
 *
 *   @return The modulation function
 */
MatrixXd CoDs::Calculate_Modulation()
{

	if (!everythingisreceived())
	{
		cout<<"Even though you forgot setting sth, you called Calculate_Modulation"<<endl;
		cout<<"State_of_system_is_set_ "<<State_of_system_is_set_<<endl;
		cout<<"State_of_surface_is_set_ "<<State_of_surface_is_set_<<endl;
		Error();
	}
	Lambda_.setIdentity();
	Lambda_Bold_.setIdentity();

	if(!Motion_Phases_[3])
	{

		qF_(0)=((F_.transpose()*N_)(0))/((F_.transpose()*F_)(0,0));
		qF_(1)=((F_.transpose()*q2_)(0))/((F_.transpose()*F_)(0,0));
		qF_(2)=((F_.transpose()*q3_)(0))/((F_.transpose()*F_)(0,0));

		if (Gammma_Threshold_<=Gamma_Value_)
		{
			double handle_Tra=exp((Gammma_Threshold_-Gamma_Value_)/(10e-5));
			Motion_Phases_[0]=true;
			Normal_velocity_robot_real_=Q_.transpose()*DXState_real_;

			if (Normal_velocity_robot_real_(0)<delta_dx_)
			{
				handle_N_=-Omega_*((N_.transpose()*DX_)(0)-(delta_dx_+nu_));
			}
			else if ((delta_dx_<Normal_velocity_robot_real_(0))&&(Normal_velocity_robot_real_(0)<0))
			{
				handle_N_=((Omega_*Omega_*(Gamma_Value_)+Omega_*nu_)/delta_dx_)*(N_.transpose()*DX_)(0)-Omega_*Omega_*(Gamma_Value_);
			}
			else if (0<=Normal_velocity_robot_real_(0))
			{
				handle_N_=-2*Omega_*((N_.transpose()*DX_)(0))-Omega_*Omega_*(Gamma_Value_);
			}

			Lambda_(0,0)=handle_N_*qF_(0);	Lambda_(0,1)=handle_N_*qF_(1);		Lambda_(0,2)=handle_N_*qF_(2);
			Lambda_(1,0)=0;					Lambda_(1,1)=1;						Lambda_(1,2)=0;
			Lambda_(2,0)=0;					Lambda_(2,1)=0;						Lambda_(2,2)=1;

			Lambda_Bold_(0,0)=(Lambda_(0,1)-1)*handle_Tra+1; 	Lambda_Bold_(0,1)=Lambda_(0,1)*handle_Tra;			Lambda_Bold_(0,2)=Lambda_(0,2)*handle_Tra;
			Lambda_Bold_(1,0)=Lambda_(1,1)*handle_Tra; 			Lambda_Bold_(1,1)=(Lambda_(1,1)-1)*handle_Tra+1;	Lambda_Bold_(1,2)=Lambda_(1,2)*handle_Tra;
			Lambda_Bold_(2,0)=Lambda_(2,1)*handle_Tra; 			Lambda_Bold_(2,1)=Lambda_(2,1)*handle_Tra;			Lambda_Bold_(2,2)=(Lambda_(2,2)-1)*handle_Tra+1;

			Phase_of_the_motion_=Phase_Free_motion;
			Motion_Phases_[1]=false;
			cout<<"Free Motion"<<endl;
		}
		else if((Gamma_Value_<Gammma_Threshold_)&&(epsilon<Gamma_Value_)&&(!Motion_Phases_[1]))
		{
			Motion_Phases_[0]=true;
			Phase_of_the_motion_=Phase_Transition;
			Normal_velocity_robot_real_=Q_.transpose()*DXState_real_;

			if (Normal_velocity_robot_real_(0)<delta_dx_)
			{
				handle_N_=-Omega_*((N_.transpose()*DX_)(0)-(delta_dx_+nu_));
				cout<<"It is going faster! "<<handle_N_<<" "<<(N_.transpose()*DX_)(0)<<endl;
			}
			else if ((delta_dx_<Normal_velocity_robot_real_(0))&&(Normal_velocity_robot_real_(0)<0))
			{
				handle_N_=((Omega_*Omega_*(Gamma_Value_)+Omega_*nu_)/delta_dx_)*(N_.transpose()*DX_)(0)-Omega_*Omega_*(Gamma_Value_);
				cout<<"It is going correct! "<<Lambda_(0,0)<<" "<<(N_.transpose()*DX_)(0)<<endl;
			}
			else if (0<=Normal_velocity_robot_real_(0))
			{
				handle_N_=-2*Omega_*((N_.transpose()*DX_)(0))-Omega_*Omega_*(Gamma_Value_);
				cout<<"It is not going to the desired direction! "<<Lambda_(0,0)<<" "<<(N_.transpose()*DX_)(0)<<endl;
			}

			Lambda_(0,0)=handle_N_*qF_(0);	Lambda_(0,1)=handle_N_*qF_(1);		Lambda_(0,2)=handle_N_*qF_(2);
			Lambda_(1,0)=0;					Lambda_(1,1)=Gain_;					Lambda_(1,2)=0;
			Lambda_(2,0)=0;					Lambda_(2,1)=0;						Lambda_(2,2)=Gain_;

			Lambda_Bold_=Lambda_;

		}
		else if  ((Gamma_Value_<=epsilon)||(Motion_Phases_[1]))
		{
			Phase_of_the_motion_=Phase_Contact;
			Motion_Phases_[1]=true;
			Normal_velocity_robot_real_=Q_.transpose()*DXState_real_;

			handle_N_=-0.2*2*Omega_*((N_.transpose()*DX_)(0))-0.01*Omega_*Omega_*(Gamma_Value_+0.2);

			Lambda_(0,0)=handle_N_*qF_(0);	Lambda_(0,1)=handle_N_*qF_(1);		Lambda_(0,2)=handle_N_*qF_(2);
			Lambda_(1,0)=0;					Lambda_(1,1)=Gain_;					Lambda_(1,2)=0;
			Lambda_(2,0)=0;					Lambda_(2,1)=0;						Lambda_(2,2)=Gain_;
/*			Lambda_(0,0)=1;					Lambda_(0,1)=0;						Lambda_(0,2)=1;
			Lambda_(1,0)=0;					Lambda_(1,1)=1;						Lambda_(1,2)=0;
			Lambda_(2,0)=0;					Lambda_(2,1)=0;						Lambda_(2,2)=1;*/
			Lambda_Bold_=Lambda_;

			cout<<"The contact is established "<<Lambda_(0,0)<<endl;
		}


	} else
	{
		Phase_of_the_motion_=Phase_Everything_is_done;
	}

	M_=Q_*Lambda_Bold_*Q_inv_;


	//	cout<<"Lambda_ "<<endl;cout<<Lambda_<<endl;
	/*	cout<<"M_ "<<endl;cout<<M_<<endl;
	cout<<"Q_ "<<endl;cout<<Q_<<endl;
	cout<<"Q_inv_ "<<endl;cout<<Q_inv_<<endl;*/
	//	everyfalse();

	return M_;

}

/**
 *   @brief Calculate the normal velocity in three dimensions
 *
 *   @return The robot's normal velocity in three dimensions
 */
VectorXd CoDs::Get_Normal_Velocity()
{
	return Q_.transpose()*DXState_real_;
}

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
 *   @brief  To check if everything has been correctly initialized or set.
 *
 *   @return A flag, if it is true, life is shiny.
 */
inline bool CoDs::everythingisreceived()
{
	/*	bool flag=true;

	flag=State_of_system_is_set_;
	flag=State_of_surface_is_set_;*/

		return ((State_of_system_is_set_)&&(State_of_surface_is_set_));

}

/**
 *   @brief  Set all the Boolens to false.
 *
 *   @return void.
 */
inline void CoDs::everyfalse()
{
	State_of_system_is_set_=false;
	State_of_system_is_set_=false;
}


/**
 *   @brief  Getting the modulated surface
 *
 *   @return Modulated surface.
 */
double CoDs::Get_Modulated_Surface()
{
	return Gamma_Value_;
}



/**
 *   @brief  Giving the state of task
 *
 *   @return Defining the state of task.
 */
bool CoDs::Is_the_task_being_done()
{

	return Motion_Phases_[3];
}


/**
 *   @brief  Giving the state of motion
 *
 *   @return Defining the state of motion.
 */

ENUM_Phase	CoDs::Which_phase_is_it()
{
	return Phase_of_the_motion_;
}





















/*

 *
 *   @brief  If something is wrong, it is activated.
 *
 *   @return void

void Gamma::Error()
{


	while(ros::ok())
	{

	}

}
 */
