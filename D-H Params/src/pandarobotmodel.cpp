#include <pandarobotmodel.h>
#include <math.h>

	
/// \brief Constructor
PandaRobotModel::PandaRobotModel()
{
	
}

PandaRobotModel::~PandaRobotModel()
{	
	
}

Eigen::Matrix4d PandaRobotModel::forwardKinematicsDH(Eigen::MatrixXd q)
{	
	const double pi = boost::math::constants::pi<double>();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T01 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T12 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T23 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T34 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T45 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T56 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T67 = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T7b = Eigen::Matrix4d::Identity();
	
	// Phi values given from q, rest are from determined DH table
	
	T01 << cos(q(0,0)), -sin(q(0,0)), 0, 0,
		   sin(q(0,0)), cos(q(0,0)), 0, 0,
		   0, 0, 1, 0.333,
		   0, 0, 0, 1;   
	
	T12 << cos(q(0,1)), -sin(q(0,1)), 0, 0,
		   0, 0, 1, 0,
		   sin(q(0,1))*-1, cos(q(0,1))*-1, 0, 0,
		   0, 0, 0, 1;
	
	// Start off with the T01 matrix as it is the base
	T = T01 * T12;
	
	T23 << cos(q(0,2)), -sin(q(0,2)), 0, 0,
		   0, 0, -1, -0.316*-1,
		   sin(q(0,2)), cos(q(0,2)), 0, 0,
		   0, 0, 0, 1;
	
	// Continue multiplying matrices until the end
	T = T * T23;
		   
	T34 << cos(q(0,3)), -sin(q(0,3)), 0, 0.0825,
		   0, 0, -1, 0,
		   sin(q(0,3)), cos(q(0,3)), 0, 0,
		   0, 0, 0, 1;
		   
	T = T * T34;	   
	
	T45 << cos(q(0,4)), -sin(q(0,4)), 0, 0.0825,
		   0, 0, 1, -0.384*1,
		   sin(q(0,4))*-1, cos(q(0,4))*-1, 0, 0,
		   0, 0, 0, 1;
		   
	T = T * T45;
		   
	T56 << cos(q(0,5)), -sin(q(0,5)), 0, 0,
		   0, 0, -1, 0,
		   sin(q(0,5))*1, cos(q(0,5))*1, 0, 0,
		   0, 0, 0, 1;
		   
	T = T * T56;
		   
	T67 << cos(q(0,6)), -sin(q(0,6)), 0, 0.088,
		   0, 0, -1, 0,
		   sin(q(0,6))*1, cos(q(0,6))*1, 0, 0,
		   0, 0, 0, 1;
		   
	T = T * T67;
		   
	T7b << 1, 0, 0, 0,
		   0, 1, 0, 0,
		   0, 0, 1, 0.107,
		   0, 0, 0, 1;
		   
	T = T * T7b;

    return T;
}

