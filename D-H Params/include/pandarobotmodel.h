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
		
		// Function to calculate the forward kinematics of the Panda manipulator using the Devanit Hartenberg method
		// Input: q - 7x1 Matrix containing acutation values for each joint in rad
		// Output: 4x4 Matrix describing the frame of the panda robot end-effector in the specified configuration q
		Eigen::Matrix4d forwardKinematicsDH(Eigen::MatrixXd q);
		
		
		
     
	private:
	
		
};

