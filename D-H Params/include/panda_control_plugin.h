#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


#include <vector>

namespace gazebo
{
  /// \brief A plugin to control the UR10 in Gazebo.
  class PandaControlPlugin : public ModelPlugin
  {
    
    public:
		
		/// \brief Constructor
		PandaControlPlugin();
	
		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	
		
     
	private:
		
		
		/// \brief Callback for Configuration Message.
		void OnJointPosMsg(ConstJointPtr &_msg);
	
		/// \brief Pointer to the model.
		physics::ModelPtr model;
	
		/// \brief Pointer to the joints.
		std::vector<physics::JointPtr> joints;
		
		/// \brief Pointer to the last link as EE
		physics::LinkPtr ee_link;
		
		/// \brief Nodes used for transport
		transport::NodePtr node;

		/// \brief A subscriber to a named topic.
		transport::SubscriberPtr sub_joints;
		
		/// \brief A publisher to a named topic.
		transport::PublisherPtr pub_pose;
    
  };
  
  	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(PandaControlPlugin);

  
}
