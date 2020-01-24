#ifndef _QEV2_DRIVE_PLUGIN_HH_
#define _QEV2_DRIVE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    // Plugin to control the QEV2 Gazebo model via driving
    class qev2Plugin : public ModelPlugin

    {
        // Constructor goes here
        public : qev2Plugin() {}

        // Load is called by Gazebo to get the plugin
        // Specify the model to attached the plugin to, as well as the .sdf describing it
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // For now a terminal message will do
            std::cerr << "Drive Plugin successfully attached to " << _model->GetName() << "\n";
        }
    };
}
#endif