#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

class AS_state_mach {
    public:
    // Constructor
    AS_state_mach();

    // Private variables
    private:
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void ebs_callback(const std_msgs::BoolConstPtr& ebs_msg);
    void res_callback(const std_msgs::BoolConstPtr& res_msg);

    ros::Subscriber joy_sub;
    ros::Subscriber ebs_sub;
    ros::Subscriber res_sub;
    ros::NodeHandle nsm;

    // State rosparam
    std::string state;

    int button_press = 0;

};

AS_state_mach::AS_state_mach() {
    // Double check we are in the right state at startup
    if(nsm.getParam("AS_State", state)) {
        if (state != "AS_Ready") {
            ROS_ERROR("Defaulting to AS_Ready");
            nsm.setParam("AS_State", "AS_Ready");
        }
    }

    // Subscribers
    joy_sub = nsm.subscribe("/joy", 10, &AS_state_mach::joy_callback, this);
    ebs_sub = nsm.subscribe("/ebs_sig", 5, &AS_state_mach::ebs_callback, this);
    res_sub = nsm.subscribe("/res_sig", 5, &AS_state_mach::res_callback, this);
}

void AS_state_mach::joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    // Handles buttons presses on the Joy message to transition states
    for(int i = 0; i <= joy_msg->buttons.size(); i++) {
        // Check which button has been pressed
        if((joy_msg->buttons[i] == 1) && (button_press == 0)) { // Catch a double press
            button_press = i + 1;
        }
    }

    // We use A to go between AS ready and Manual
    if ((button_press == 1) && (state == "AS_Ready")) {
        // Go to the manual state
        ROS_INFO_NAMED("AS_state_mach", "Switching to manual mode");
        nsm.setParam("AS_State", "AS_Manual");
        state = "AS_Manual";
        button_press = 0; // Reset the button press
    }

    if ((button_press == 2) && (state == "AS_Manual")) {
        // Go to the manual state
        ROS_INFO_NAMED("AS_state_mach", "Switching to ready mode");
        nsm.setParam("AS_State", "AS_Ready");
        state = "AS_Ready";
        button_press = 0;
    }

    // Use the X button to go from AS ready to driving
    if ((button_press == 3) && (state == "AS_Ready")) {
        // Go to the manual state
        ROS_INFO_NAMED("AS_state_mach", "Switching to AS Driving!");
        nsm.setParam("AS_State", "AS_Driving");
        state = "AS_Driving";
        button_press = 0;
    }

    // Fill here for transition to AS Finished
    // if vehicle speed 0 and we arent in an emergency transition to AS_Finished

    // Use X to go back to AS Ready when done
    if ((button_press == 3) && (state == "AS_Finished")) {
        // Go to the manual state
        ROS_INFO_NAMED("AS_state_mach", "AS finished, transitioning to ready");
        nsm.setParam("AS_State", "AS_Ready");
        state = "AS_Ready";
        button_press = 0;
    }    

}

void AS_state_mach::ebs_callback(const std_msgs::BoolConstPtr& ebs_msg) {
    // If we get an EBS notification, go to the emergency state
    if (ebs_msg->data) {
        nsm.setParam("AS_State", "AS_Emergency");
        ROS_ERROR("EBS activated. Entering emergency state!");
        state = "AS_Emergency";
    }
}

void AS_state_mach::res_callback(const std_msgs::BoolConstPtr& res_msg) {
    // If we get an EBS notification, go to the emergency state
    if (res_msg->data) {
        nsm.setParam("AS_State", "AS_Emergency");
        ROS_ERROR("RES tripped. Entering emergency state!");
        state = "AS_Emergency";
    }
}

// Main here
int main (int argc, char **argv) {
    // Init
    ros::init(argc, argv, "AS_State_Machine");

    // Run and spin
    AS_state_mach as_state_machine;

    ros::spin();
}

