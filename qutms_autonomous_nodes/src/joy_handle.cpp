#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <tf/transform_datatypes.h>
#include <map>
#include <string>
#include <math.h>

class Joy_to_Ackermann {
    public:
    Joy_to_Ackermann();

    void joy_callback(const sensor_msgs::JoyConstPtr joy_msg);
    void joy_to_message(void);

    private:
    ros::NodeHandle nj;
    ros::Publisher ack_pub;
    ros::Publisher ebs_pub;
    ros::Publisher res_pub;
    ros::Subscriber joy_sub;
    ackermann_msgs::AckermannDrive ack_msg;
    std_msgs::Bool ebs_msg;
    std_msgs::Bool res_msg;

    int accel = 10; // m/s^2
    int turn = M_PI_2; // Maximum and minimum turn angle
    double accel_gain, angle_gain;
    bool ebs = false;
    bool res = false;
    
};

Joy_to_Ackermann::Joy_to_Ackermann() {
    // Setup publishers and subscribers
    ack_pub = nj.advertise<ackermann_msgs::AckermannDrive>("/qev/vel_cmd", 10);
    ebs_pub = nj.advertise<std_msgs::Bool>("/qev/ebs_sig", 10, true);
    res_pub = nj.advertise<std_msgs::Bool>("/qev/res_sig", 10, true);
    
    // Subscribers here
    joy_sub = nj.subscribe("/joy", 10, &Joy_to_Ackermann::joy_callback, this);

}

void Joy_to_Ackermann::joy_to_message(void) {
    // Map all values in and publish
    ack_msg.acceleration = accel*accel_gain;
    ack_msg.steering_angle = turn*angle_gain;

    ack_pub.publish(ack_msg);
}

void Joy_to_Ackermann::joy_callback(const sensor_msgs::JoyConstPtr joy_msg) {
    // Map acceleration and angle gain to joystick messages
    double left_val = joy_msg->axes[0];
    double right_val = joy_msg->axes[1];

    // Keep the value at 0 if no input is present
    if((left_val == 0) && (right_val == 0)) {
        Joy_to_Ackermann::angle_gain = 0;
    }
    
    // Assign
    if (left_val == 0) {
        // Assign the right value
        Joy_to_Ackermann::angle_gain = right_val;
    } else if (right_val == 0) {
        // Assign the left value
        Joy_to_Ackermann::angle_gain = left_val;
    }

    // Map the acceleration gain
    Joy_to_Ackermann::accel_gain = joy_msg->axes[3];

    // Activate if the EBS is pressed
    int b_press = joy_msg->buttons[1];
    int x_press = joy_msg->buttons[2];

    // Toggle the EBS
    if ((b_press == 1) && (ebs == false)) {
        ebs = true;
        ebs_msg.data = ebs;
        ebs_pub.publish(ebs_msg);
    }
    if ((b_press == 1) && (ebs == true)) {
        ebs = false;
        ebs_msg.data = ebs;
        ebs_pub.publish(ebs_msg);
    }

    // Toggle the RES
    if ((x_press == 1) && (res == false)) {
        res = true;
        res_msg.data = res;
        res_pub.publish(res_msg);
    }
    if ((x_press == 1) && (res == true)) {
        res = false;
        res_msg.data = res;
        res_pub.publish(res_msg);
    }    

}

int main(int argc, char** argv) {
    // Initialise
    ros::init(argc, argv, "Joy_Controller");

    // Class set
    Joy_to_Ackermann joy_to_ack;

    // Rate set
    ros::Rate rate(10); // 10Hz

    // Main loop
    while(ros::ok()) {
        // Spin once
        ros::spinOnce();

        // Set the Ackermann message
        joy_to_ack.joy_to_message();

        // Sleep
        rate.sleep();
    }
}