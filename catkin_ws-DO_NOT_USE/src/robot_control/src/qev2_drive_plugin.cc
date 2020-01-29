#ifndef _QEV2_DRIVE_PLUGIN_HH_
#define _QEV2_DRIVE_PLUGIN_HH_

#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

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
            // Safety check
            if (_model->GetJointCount() == 0)
            {
                std::cerr << "Invalid joint count, drive plugin not loaded\n";
                return;
            }

            // Store the model pointer for convenience.
            this->model = _model;

            // RIGHT WHEEL STEERING ROTATION

            // Get the front right wheel joint
            this->joint = _model->GetJoints()[0];

            // Setup a PID controller with a 0.1 gain
            this->pid = common::PID(0.1, 0, 0);

            // Apply the gain specified
            this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

            // LEFT WHEEL STEERING 

            // Get the front left wheel
            this->joint2 = _model->GetJoints()[1];

            // Apply the gain
            this->model->GetJointController()->SetVelocityPID(this->joint2->GetScopedName(), this->pid);

            // REAR WHEELS

            // Get the rear wheel joints
            this->joint3 = _model->GetJoints()[2];
            this->joint4 = _model->GetJoints()[3];

            this->model->GetJointController()->SetVelocityPID(this->joint3->GetScopedName(), this->pid);
            this->model->GetJointController()->SetVelocityPID(this->joint4->GetScopedName(), this->pid);

            // Default to zero velocity
            double velocity = 0;

            // Check that the velocity element exists, then read the value
            if (_sdf->HasElement("velocity")) {
                velocity = _sdf->Get<double>("velocity");
            }

            // Set the joint's target velocity
            this->SetVelocity(velocity);

            // Create a node to handle message transporting
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->Name());
            
            // Create a topic to advertise on
            std::string topic_name = "~/" + this->model->GetName() + "/vel_cmd";

            // Subscribe to this topic and create a callback to register this
            this->sub = this->node->Subscribe(topic_name, &qev2Plugin::OnMsg, this); 

            // Start ROS
            if (!ros::isInitialized()) {
                // Initialize node
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            // Create the node
            this->ros_node.reset(new ros::NodeHandle("gazebo_client"));

            // Create a subscribe to a topic
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/vel_cmd", 1, boost::bind(&qev2Plugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->ros_queue); 
            this->ros_sub = this->ros_node->subscribe(so);

            // Start the queue helper
            this->ros_queue_thread = std::thread(std::bind(&qev2Plugin::Queue_Thread, this));
        }

        // Set the velocity of QEV2
        // New target velocity
        public: void SetVelocity(const double &_vel) {
            // Set the joint's target velocity.
            this->model->GetJointController()->SetVelocityTarget(this->joint3->GetScopedName(), _vel);
            this->model->GetJointController()->SetVelocityTarget(this->joint4->GetScopedName(), _vel);
        }

        // Handle an incoming ROS message
        public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg) {
            this->SetVelocity(_msg->data);
        }

        private: void Queue_Thread() {
            static const double time_out = 0.01;
            while(this->ros_node->ok()) {
                this->ros_queue.callAvailable(ros::WallDuration(time_out));
            }
        }

        // Handle the incoming message
        private: void OnMsg(ConstVector3dPtr &_msg) {
            this->SetVelocity(_msg->x());
        }

        // A node used for transport
        private: transport::NodePtr node;

        // A subscriber to a named topic.
        private: transport::SubscriberPtr sub;

        // Pointer to the model.
        private: physics::ModelPtr model;

        // Pointer to the joint.
        private: physics::JointPtr joint;
        private: physics::JointPtr joint2;
        private: physics::JointPtr joint3;
        private: physics::JointPtr joint4;

        // A PID controller for the joint.
        private: common::PID pid;

        // ROS class members
        // ROS node for transport
        private: std::unique_ptr<ros::NodeHandle> ros_node;

        // Subscriber
        private: ros::Subscriber ros_sub;

        // Callback
        private: ros::CallbackQueue ros_queue;

        // Open a thread to run the queue
        private: std::thread ros_queue_thread;
    };

    GZ_REGISTER_MODEL_PLUGIN(qev2Plugin)
}
#endif