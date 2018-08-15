[![Build Status](https://travis-ci.org/sinamr66/CoDs_SDK.svg?branch=master)](https://travis-ci.org/sinamr66/CoDs_SDK)
# Dynamical system based compliant contact controller

Catkin package implementing DS based contact controller which was proposed in

```
Mirrazavi Salehian, S. S. and Billard, A. (2018) A Dynamical System Based Approach for Controlling Robotic Manipulators During Non-contact/Contact Transitions. IEEE Robotics and Automation Letters (RA-L).
```
You can find a use case study of this here:
[https://www.youtube.com/embed/fhfBBMH4XVg](https://www.youtube.com/embed/fhfBBMH4XVg)

# Dependences 

- Eigen http://eigen.tuxfamily.org/index.php?title=Main_Page

# Features:
- Controlling for contact and impact.
  - Making sure that the contact is going to be stable and the robot does not bounce on the surface.
- Leaving the surface or staying on the it at desired points.


## Documentation
You can get some basic source code documentation by running doxygen.

```
sudo apt-get install doxygen
roscd CoDs_SDK
doxygen Doxyfile
```

# How to run 
### Initialization:
1- Initialize the modulation function:
```
void initialize(int Dimen_state,double delta_dx,double F_d,double Gammma_free_motion, bool define_desired_contact_point, bool define_desired_leaving_point);
```

### In the loop:
1- Set the state of the gamma function, which conveys the notion of distance between the robot's end-effector and the contact surface
```
void Set_Gamma(double Gamma,VectorXd Normal,VectorXd q2,VectorXd q3,VectorXd Point_on_surface);
```
2- Set the state of the robot
```
  void Set_State(VectorXd State,VectorXd DState,VectorXd DState_real,VectorXd Original_Dynamic);
```
3- Set the location of the desired contact point
```
  void Set_Contact_point(VectorXd Contact_point);
```
4- Set the location of the desired leaving point and the location of the target in space!
```
  VectorXd Set_Leaving_point(VectorXd Leaving_point,VectorXd X_Target);
```
5- Calculate the modulation function 
```
MatrixXd Calculate_Modulation();
```
6- Calculate the modulation function 
```
MatrixXd Calculate_Modulation();
```
**Note 1: Most of the variables are accessible by get_ functions see [CoDs_SDK.h](https://github.com/sinamr66/CoDs_SDK/blob/master/include/CoDs.h)**

### Contact information
For more information contact Sina Mirrazavi.
