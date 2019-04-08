#ifndef _EV3_PLUGIN_HH_
#define _EV3_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class EV3Plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: EV3Plugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }
      
      this->model = _model;

      for (const physics::JointPtr& ptr : model->GetJoints())
      {
          if (ptr->GetScopedName() == "ev3::left_wheel")
              this->left_wheel = ptr;
          else if (ptr->GetScopedName() == "ev3::right_wheel")
              this->right_wheel = ptr;
      }
    }

    private:
        physics::ModelPtr model;

        physics::JointPtr left_wheel;
        physics::JointPtr right_wheel;

        common::PID pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(EV3Plugin)
}
#endif
