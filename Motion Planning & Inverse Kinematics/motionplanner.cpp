#include <motionplanner.h>
#include <math.h>
#include <algorithm>
	
/// \brief Constructor
MotionPlanner::MotionPlanner(GazeboController* controller)
{
	
	//Hand over pointer to Gazebo controller
	mp_controller = controller;
	
	//Define joint limits w.r.t. Gazebo simulation
	Eigen::Matrix<double,7,1> q_min;
	q_min << 	-2.9,
				-1.76,
				-2.9,
				-3.07,
				-2.9,
				-0.02,
				-2.9;
				
	Eigen::Matrix<double,7,1> q_max;
	q_max << 	2.9,
				1.76,
				2.9,
				0,
				2.9,
				3.75,
				2.9;
	
	//Setup random number generators
    m_gen = std::mt19937(); 
    m_dis1 = std::uniform_real_distribution<>(q_min(0), q_max(0));
    m_dis2 = std::uniform_real_distribution<>(q_min(1), q_max(1));
    m_dis3 = std::uniform_real_distribution<>(q_min(2), q_max(2));
    m_dis4 = std::uniform_real_distribution<>(q_min(3), q_max(3));
    m_dis5 = std::uniform_real_distribution<>(q_min(4), q_max(4));
    m_dis6 = std::uniform_real_distribution<>(q_min(5), q_max(5));
    m_dis7 = std::uniform_real_distribution<>(q_min(6), q_max(6));
}

MotionPlanner::~MotionPlanner()
{	
	
}

std::vector<Eigen::MatrixXd> MotionPlanner::planMotion(Eigen::MatrixXd q_start,Eigen::MatrixXd q_goal)
{	
	int max_nodes = 5000;
	int num_nodes = 1;
	double step_length = 0.5;
	int n_goal = 5;
	TreeNode *root;
	root = new TreeNode(q_start);
	root->parent = NULL;
	int iterCount = 0;
	double move_rate = 0.5;
	double move_rate_epsilon = 0.5;
	std::vector<Eigen::MatrixXd> path;
	std::vector<Eigen::MatrixXd> preReversePath;
	std::cout<< "Can start from a valid position.\n\n";
	while (num_nodes < max_nodes) {
		std::cout<< "Node count: " << num_nodes << "\n\n";
		// Initial condition
		Eigen::MatrixXd q_sample = q_goal;
		// Taken from point 2 in Practical 8
		if (iterCount % n_goal != 0) {
			// Calls a random sample
			q_sample = MotionPlanner::drawRandomConfig();
		}
		TreeNode * sampleNode;
		sampleNode = new TreeNode(q_sample);			
		TreeNode *nearest = getNearestNode(root, sampleNode);
		TreeNode *newNode = getNewNode(nearest, sampleNode, move_rate);
		if (nearest->vec == newNode->vec) {
			iterCount++;
			continue;
		}
		bool blockedPath = isThereObstacle(nearest->vec, newNode->vec, move_rate_epsilon);
		// Checks out as no collision
		if (!blockedPath) {
			nearest->addChild(newNode);
			root->addChild(nearest);
			// If arriving at the final 
			if (newNode->vec == q_goal) {
				TreeNode *curr = newNode; 
				while (curr->parent != NULL) {
					preReversePath.push_back(curr->vec);
					curr = curr->parent;
				}
				break;
			}
		}
	}
	// Have to reverse the initial path vector since it's added
	// joint values are in reverse
	std::reverse(preReversePath.begin(), preReversePath.end());
	return path;
}

Eigen::MatrixXd MotionPlanner::drawRandomConfig()
{	
	Eigen::MatrixXd q;
	q.resize(7,1);
	
	q << 	m_dis1(m_gen),
			m_dis2(m_gen),
			m_dis3(m_gen),
			m_dis4(m_gen),
			m_dis5(m_gen),
			m_dis6(m_gen),
			m_dis7(m_gen);
			
	return q;
	
}

bool MotionPlanner::getCollisionState(Eigen::MatrixXd q_check)
{	
	
	if(q_check.rows() != 7 || q_check.cols() != 1)
	{
		std::cout << "Wrong dimensions of robot configuration!" << std::endl;
		return false;
	}
	
	mp_controller->setRobotConfiguration(q_check);
	
	//Make sure the robot moved in simulation and the EE frame updated
	while(!mp_controller->isFrameValid())
	{
			
	}
			
	return mp_controller->getCollisionState();
	
}

TreeNode * MotionPlanner::getNearestNode(TreeNode *root, TreeNode *sample) {
	// Use a simple BFS algorithm to traverse all nodes
	std::queue<TreeNode *> queue;
	queue.push(root);
	double lowest_dist = INFINITY;
	double curr_dist;
	TreeNode *closest;
	TreeNode *curr;
	
	while (!queue.empty()) {
		curr = queue.front();
		queue.pop();
		curr_dist = (curr->vec - sample->vec).norm();
		// Update the lowest distance and closest node if this is satisfied
		if (curr_dist < lowest_dist && curr_dist != 0) {
			closest = curr;
			lowest_dist = curr_dist;
		}
		for (int i = 0; i < (curr->num_children - 1); i++) {
			queue.push(curr->children[i]);
		}
	}
	return closest;
}

TreeNode * MotionPlanner::getNewNode(TreeNode *nearest, TreeNode *sample, double move_rate) {
	TreeNode *xNew;
	xNew->parent = nearest;
	xNew->num_children = 0;
	
	for (int i = 0; i < 7; i++) {
		xNew->vec(i,0) = nearest->vec(i,0) + move_rate*(sample->vec(i,0) - nearest->vec(i,0));
	}
	
	return xNew;
}

bool MotionPlanner::isThereObstacle(Eigen::MatrixXd start, Eigen::MatrixXd end, double epsilon) {
	// Checks a linear path from the start to end to checks for 
	// obstacles in that path, at a certain rate (epsilon)
	Eigen::Matrix<double,7,1> temp;
	double num_partitions = (start - end).norm() / epsilon;
	
	for (int i = 0; i < num_partitions; i++) {
		// Adds the epsilon value for each partition of the line
		for (int x = 0; x < 7; x++) {
			temp(x,0) = (i*epsilon) + start(x,0);
		}
		if (!MotionPlanner::getCollisionState(temp)) {
			return true;
		}
	}
	// If no false is returned, then there are no collisions along the 
	// path from start to end
	return false;
}


