#pragma once

#include <Eigen/Dense>


/// \brief This class provides functionalities to calculate the kinematics for the Panda Manipulator
/// This class is empty and needs to be implemented throughout the different assignment questions
class PandaRobotModel
{
    
    public:
		
		//Constructor and Destructor
		PandaRobotModel();
		~PandaRobotModel();
		
		// Function to calculate the forward kinematics of the Panda manipulator using the Product of Exponentials Formula
		// Input: q - 7x1 Matrix containing acutation values for each joint in rad
		// Output: 4x4 Matrix describing the frame of the panda robot end-effector in the specified configuration q
		Eigen::Matrix4d forwardKinematicsPoE(Eigen::MatrixXd q);
		
		// Function to calculate Space Jacobian of the Panda manipulator
		// Input: q - 7x1 Matrix containing acutation values for each joint in rad
		// Output: 6x7 Space Jacobian Matrix relating joint velocities to end-effector twist
		Eigen::MatrixXd getSpaceJacobian(Eigen::MatrixXd q);
		
		// Function to calculate Body Jacobian of the Panda manipulator
		// Input: q - 7x1 Matrix containing acutation values for each joint in rad
		// Output: 6x7 Body Jacobian Matrix relating joint velocities to end-effector twist
		Eigen::MatrixXd getBodyJacobian(Eigen::MatrixXd q);
		
		// Function to obtain singular values, singular vectors and reciprocal condition number of any Jacobian matrix
		// Input: J - nx7 Jacobian Matrix to be evaluated
		// Output: sigma - nx1 column Vector of singular values of the specified Jacobian 
		// Output: U - nxn Matrix containing the singular vectors of the specified Jacobian as columns (nth column corresponds to nth singular value in sigma)
		// Output: rcond - reciprocal condition number of the specified Jacobian Matrix
		void getSingularValues(Eigen::MatrixXd &sigma, Eigen::MatrixXd &U, double &rcond, Eigen::MatrixXd J);
		
     
	private:
	
		
};

