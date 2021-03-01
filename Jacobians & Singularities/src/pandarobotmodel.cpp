#include <pandarobotmodel.h>


	
/// \brief Constructor
PandaRobotModel::PandaRobotModel()
{
	
}

PandaRobotModel::~PandaRobotModel()
{	
	
}

Eigen::Matrix4d PandaRobotModel::forwardKinematicsPoE(Eigen::MatrixXd q)
{	
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    return T;
	
}

Eigen::MatrixXd PandaRobotModel::getSpaceJacobian(Eigen::MatrixXd q)
{
	
    Eigen::MatrixXd spaceJ(6, q.size());
    
	spaceJ.setZero();

    return spaceJ;
}

Eigen::MatrixXd PandaRobotModel::getBodyJacobian(Eigen::MatrixXd q)
{	
	Eigen::MatrixXd bodyJ(6, q.size());
    
	bodyJ.setZero();

    return bodyJ;
	
}

void PandaRobotModel::getSingularValues(Eigen::MatrixXd &sigma, Eigen::MatrixXd &U, double &rcond, Eigen::MatrixXd J)
{	
	
	sigma.resize(J.rows(),1);
	sigma.setZero();
	
	U.resize(J.rows(),J.rows());
	U.setZero();
	
	rcond = 0;
	
}


