#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// Controller for the autonomous QEV2 vehicle (in sim)

class QEV_Auto_Control {

    public:
    QEV_Auto_Control();

    void gain_handle(void);
    void gain_callback(const std_msgs::Float32ConstPtr& gain_msg);
    void lap_callback(const std_msgs::Int64ConstPtr& lap_msg);
    void blue_cone_callback(const std_msgs::BoolConstPtr& blue_msg);
    void yllw_cone_callback(const std_msgs::BoolConstPtr& yllw_msg);

    private:
    ros::NodeHandle ng;
    ros::Publisher drive_pub;
    ros::Subscriber lap_sub;
    ros::Subscriber gain_sub;
    ros::Subscriber blue_sub;
    ros::Subscriber yllw_sub;
    ackermann_msgs::AckermannDriveStamped ack_msg;

    double steering_gain, steer_val, accel_val, speed_val;
    int lap_num;
    bool no_blue, no_yellow;

    // Storage for mission name
    std::string mission_name;

};

QEV_Auto_Control::QEV_Auto_Control() {
    // Setup publisher and subscribers
    drive_pub = ng.advertise<ackermann_msgs::AckermannDriveStamped>("/qev/drive_cmd", 5);

    gain_sub = ng.subscribe("/qev/move_point", 10, &QEV_Auto_Control::gain_callback, this);
    lap_sub = ng.subscribe("/qev/lap_count", 10, &QEV_Auto_Control::lap_callback, this);
    blue_sub = ng.subscribe("/qev/blue_count", 10, &QEV_Auto_Control::blue_cone_callback, this);
    yllw_sub = ng.subscribe("/qev/yellow_count", 10, &QEV_Auto_Control::yllw_cone_callback, this);

    // Set default values here
    steer_val = M_PI_2; // Maximum steering angle

    // Grab the mission name
    if(ng.getParam("/mission_name", mission_name)) {
        ng.getParam("/mission_name", mission_name);
        std::cout << "Mission name is " << mission_name << std::endl;
    } else {
        ROS_ERROR("Unable to retrieve mission name");
    }

}

void QEV_Auto_Control::gain_handle(void) {

    // If the gain value is small, set it to 0
    if (steering_gain < 0.05) {
        steering_gain = 0;
    }
    // Apply the calculated gain to the steering angle value
    if((!no_blue) && (!no_yellow)) {
        ack_msg.drive.steering_angle = steering_gain*steer_val;
    } 

    // If we find no blue, turn left
    if (no_blue) {
        ack_msg.drive.steering_angle = 0.2*steer_val;
    }

    // if we find no yellow, turn right
    if (no_yellow) {
        ack_msg.drive.steering_angle = -0.2*steer_val;        
    }

    // If we are doing acceleration, decelerate when we reach the finish line
    if(mission_name == "acceleration") {
        if (lap_num < 1) {
            ack_msg.drive.acceleration = 20;
            ack_msg.drive.speed = 32;
        } else {
            ROS_INFO("Mission finished, stopping...");
            ack_msg.drive.acceleration = -30;
            ack_msg.drive.speed = 0;
            ack_msg.drive.jerk = 0;
        }
    }
    
    if(mission_name == "trackdrive") {
        if (lap_num < 10) {
            ack_msg.drive.acceleration = 0.2;
            ack_msg.drive.speed = 0.05;
        } else {
            ROS_INFO("Mission finished, stopping...");
            ack_msg.drive.acceleration = -20;
            ack_msg.drive.speed = 0;
            ack_msg.drive.jerk = 0;            
        }
    }

    // Header stuff
    ack_msg.header.stamp = ros::Time::now();

    // Publish
    drive_pub.publish(ack_msg);

}

void QEV_Auto_Control::gain_callback(const std_msgs::Float32ConstPtr& gain_msg) {
    // Assign the gain value
    QEV_Auto_Control::steering_gain = gain_msg->data;
}

void QEV_Auto_Control::lap_callback(const std_msgs::Int64ConstPtr& lap_msg) {
    // Update the lap counter
    QEV_Auto_Control::lap_num = lap_msg->data;
}

void QEV_Auto_Control::blue_cone_callback(const std_msgs::BoolConstPtr& blue_msg) {
    // Assign data
    QEV_Auto_Control::no_blue = blue_msg->data;
}

void QEV_Auto_Control::yllw_cone_callback(const std_msgs::BoolConstPtr& yllw_msg) {
    // Assign data
    QEV_Auto_Control::no_yellow = yllw_msg->data;
}

int main(int argc, char **argv) {
    // Initialise
    ros::init(argc, argv, "Auto_Controller");

    // Class initialise
    QEV_Auto_Control auto_control;

    // Rate set
    ros::Rate rate(30); // 5Hz

    while(ros::ok()) {
        // Spin once
        ros::spinOnce();

        // Publish the gain value
        auto_control.gain_handle();

        // Sleep
        rate.sleep();
    }
}