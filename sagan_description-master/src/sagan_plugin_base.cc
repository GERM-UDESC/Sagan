#ifndef _SAGAN_PLUGIN_HH_
#define _SAGAN_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

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
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(SaganPlugin)
}
#endif
