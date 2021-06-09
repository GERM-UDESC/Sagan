#ifndef _SAGAN_PLUGIN_HH_
#define _SAGAN_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>
#include <iostream>
#include <time.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class SaganPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: SaganPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      // Just output a message for now
      std::cerr << "\nThe Sagan plugin is attach to model[" << model->GetName() << "]\n";
    
      this->model = model;

      this->pid_motor = common::PID(0.1, 0, 0);

      std::cerr << this->model->GetChildCount() << "\n";

      std::cerr << this->model->GetJointCount() << "\n";
      std::cerr << this->model->GetJointCount() << "\n";

      //Get Joint Pointer
      this->BLW_joint = this->model->GetJoints()[0];
      this->FLW_joint = this->model->GetJoints()[1];
      this->BRW_joint = this->model->GetJoints()[2];
      this->FRW_joint = this->model->GetJoints()[3];
/*
      //Apply P controller to the joints
      this->model->GetJointController()->SetVelocityPID(this->BLW_joint->GetScopedName(), this->pid_motor);
      this->model->GetJointController()->SetVelocityPID(this->FLW_joint->GetScopedName(), this->pid_motor);
      this->model->GetJointController()->SetVelocityPID(this->BRW_joint->GetScopedName(), this->pid_motor);
      this->model->GetJointController()->SetVelocityPID(this->FRW_joint->GetScopedName(), this->pid_motor);*/

    }

  //Sagan parameters
  private: double wheel_radius = 0.2;

  private: double wheel_sep = 0.3;

  //Sagan pointers
	private: physics::ModelPtr model;

	private: physics::JointPtr BLW_joint;

	private: physics::JointPtr FLW_joint;

	private: physics::JointPtr BRW_joint;

	private: physics::JointPtr FRW_joint;

	private: common::PID pid_motor;

	//Command variables
	private: double BLW_cmd = 0;

	private: double FLW_cmd = 0;

	private: double BRW_cmd = 0;

	private: double FRW_cmd = 0;

	private: double vel_linear_cmd = 0;

	private: double vel_angular_cmd = 0;

	//Sagan data
	private: double sagan_state[6] = {0,0,0,0,0,0};

	private: double sagan_vel[6] = {0,0,0,0,0,0};


  };
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(SaganPlugin)
}
#endif