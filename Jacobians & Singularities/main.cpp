#include <gazebocontroller.h>
#include <pandarobotmodel.h>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <algorithm>

// Main code of the assignment framework
int main(int _argc, char **_argv)
{
	
	// Create Gazebo controller and Panda robot model objects
	GazeboController controller;
	PandaRobotModel robot_model;
	
	
	// Read configurations from csv file
	std::ifstream configFile("../configs.csv");
	
	// Make sure the file is open
    if(!configFile.is_open()) throw std::runtime_error("Could not open file");
    
    //Create data structure to store configurations
    std::vector<Eigen::MatrixXd> configs;
    // Read data, line by line
    std::string line;
    double value;
    while(std::getline(configFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);
        
        // Keep track of the current column index
        int colIdx = 0;
        
        
        Eigen::Matrix<double,7,1> config;
        // Extract each integer
        while(ss >> value){
            
            // Add the current value to the configuration vector
            config(colIdx) = value;
            
            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();
            
            // Increment the column index
            colIdx++;
        }
        configs.push_back(config);
    }

    // Close file
    configFile.close();
    
			
	//Run through all of the configurations that were specified in the file
	Eigen::Matrix<double,7,1> q;
	for(int i = 0; i < configs.size(); i++)
	{
		q = configs.at(i);
		
		//Move the robot to desired configuration
		controller.setRobotConfiguration(q);
		
		//Make sure the robot moved in simulation and the EE frame updated
		while(!controller.isFrameValid())
		{
		}
		
		//Calculate EE Frame with PoE
		Eigen::Matrix4d ee_frame_PoE = robot_model.forwardKinematicsPoE(q);
		
		//Get the EE Frame from the simulated robot
		Eigen::Matrix4d ee_frame_sim = controller.getCurrentRobotEEFrame();
		
		//Output frames from calculations and simulation to compare them
		std::cout << "EE frame calculated using PoE:" << std::endl;
		std::cout << ee_frame_PoE << std::endl << std::endl;
		
		std::cout << "EE frame from simulated robot:" << std::endl;
		std::cout << ee_frame_sim << std::endl << std::endl;
		
		//Spawn and move a colored frame to the calculated EE pose
		controller.moveEEFrameVis(ee_frame_PoE);
		
		std::cout << "Press any key to continue" << std::endl;
		std::getchar();
		
		
		//Delete the frame at the EE pose
		controller.deleteEEFrameVis();
		
		//Calculate Body Jacobian
		Eigen::MatrixXd J_b = robot_model.getBodyJacobian(q);
		std::cout << "Body Jacobian:" << std::endl;
		std::cout << J_b << std::endl << std::endl;
		
		//Calculate singular values
		Eigen::MatrixXd sigma;
		Eigen::MatrixXd U;
		double rcond;
		robot_model.getSingularValues(sigma,U,rcond,J_b);
		
		std::cout << "Reciprocal condition number:" << std::endl;
		std::cout << rcond << std::endl << std::endl;
		
		std::cout << "Press any key to continue" << std::endl;
		std::getchar();
		
		//Now only consider translational part of Jacobian
		Eigen::MatrixXd J_b_t = J_b.block(3,0,3,7);
		robot_model.getSingularValues(sigma,U,rcond,J_b_t);
		std::cout << "Translation body Jacobian:" << std::endl;
		std::cout << J_b_t << std::endl << std::endl;
		std::cout << "Singularvalues:" << std::endl;
		std::cout << sigma << std::endl << std::endl;
		
		//Draw manipulability ellipsoid
		controller.drawManipulabilityEllipsoid(ee_frame_PoE, sigma, U);
		
		std::cout << "Press any key to continue" << std::endl;
		std::getchar();
		
		// Delete manipulability ellipsoid
		controller.deleteManipulabilitzEllipsoid();
	}
	
	// Move robot to initial configuration
	q.setZero();
	controller.setRobotConfiguration(q);
	
	//Make sure the robot moved in simulation and the EE frame updated
	while(!controller.isFrameValid())
	{
	}
    
    
    return 0;
}
