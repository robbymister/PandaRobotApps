#include <panda_control_plugin.h>

namespace gazebo
{
	
	/// \brief Constructor
	PandaControlPlugin::PandaControlPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    void PandaControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
		
		
		// Check if the plugin is connected to the correct model
		std::cerr << "\nThe Panda plugin is attach to model[" << _model->GetName() << "]\n";
		
		std::cout << "Number of joints is " << _model->GetJointCount() << std::endl;
        
        // Safety check
		if (_model->GetJointCount() == 0)
		{
			std::cerr << "Invalid joint count, Panda plugin not loaded\n";
			return;
		}

		// Store the model pointer for convenience.
		this->model = _model;
		
		// Get the joints.
		this->joints.push_back(_model->GetJoint("panda_joint1"));
		this->joints.push_back(_model->GetJoint("panda_joint2"));
		this->joints.push_back(_model->GetJoint("panda_joint3"));
		this->joints.push_back(_model->GetJoint("panda_joint4"));
		this->joints.push_back(_model->GetJoint("panda_joint5"));
		this->joints.push_back(_model->GetJoint("panda_joint6"));
		this->joints.push_back(_model->GetJoint("panda_joint7"));
		
		//Get the distal link
		this->ee_link = _model->GetLink("panda_link7");
		
		//Set up node for communication
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init(this->model->GetWorld()->Name());
		

		// Subscribe to the joint topic, and register a callback
		this->sub_joints = this->node->Subscribe("/robot_interface/joint_pos", &PandaControlPlugin::OnJointPosMsg, this);
		
		// Publish to the pose topic
		this->pub_pose = this->node->Advertise<gazebo::msgs::Pose>("/robot_interface/ee_pose");
		
		
		
    }
	
	void PandaControlPlugin::OnJointPosMsg(ConstJointPtr &_msg)
    {
		for(int i = 0; i < 7; i++)
		{
			this->joints.at(i)->SetPosition(0,_msg->angle(i));
		}
		
		//Publish updated ee-pose
		gazebo::msgs::Pose msg;
		
		gazebo::msgs::Set(&msg,ee_link->WorldPose());
		
		pub_pose->WaitForConnection();
		pub_pose->Publish(msg);
    }
	


    
  
}
