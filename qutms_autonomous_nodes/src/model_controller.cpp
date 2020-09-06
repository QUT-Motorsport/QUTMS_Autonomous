#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// Takes input from a controller and creates a corresponding drive message to a model
// Can alternatively utilize arrow key input

#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_E 0x65

// Handles arrow key input to use for simulation as a manual driving mode

// Setup for keyboard input
int kfd = 0;
struct termios cooked, raw;

// // Set values initially
// int turn = 0;
// int accel = 0;


void SigIntHandler(int sig) {    
    // Tests for shutdown, activates the EBS
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    // accel = 0;
    // turn = 0;
    ros::shutdown();
    exit(0);

}

// Main here
int main(int argc, char **argv) {
    // Initialise
    ros::init(argc, argv, "QEV_Manual_Drive", ros::init_options::NoSigintHandler);
    
    // Only need 2 node handles
    ros::NodeHandle na;
    // Publishers go here
    ros::Publisher forward = na.advertise<ackermann_msgs::AckermannDriveStamped>("/qev/drive_cmd", 5, true);

    // Define messages
    ackermann_msgs::AckermannDriveStamped drive_msg;

    // Rate set
    ros::Rate rate(10); // 10Hz

    // Set values initially
    double turn = 0;
    double accel = 0;

    // Check for shutdown
    signal(SIGINT, SigIntHandler);

    // Grab keyboard input
    char c;
    bool clean = false;
    
    // Console grab
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    ROS_INFO("Manual Drive entered. Use arrow keys to move, E to activate EBS");

    while(ros::ok()) {
        // Retrieve the keyboard event
        if(read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }

        // ROS_INFO("Value: 0x%02X\n", c);

        switch(c) {
            // Left command
            case KEYCODE_L:
            // Check if the value is already non-zero. If so, increase it
            ROS_INFO("Commanding left");
            if(turn <= M_PI/2) {
                turn = turn + M_PI/18; // Increase by 10 degrees
            }
            clean = true;
            break;

            // Right command
            case KEYCODE_R:
            // Check if the value is already non-zero. If so, decrease it
            ROS_INFO("Commanding right");            
            if (turn >= -M_PI/2) {
                turn = turn - M_PI/18;
            }
            clean = true;
            break;

            // Forward command
            case KEYCODE_U:
            // Check if the value is already non-zero and below 4. If so, increase it
            ROS_INFO("Commanding forward");            
            if(accel <= 10) {
                accel++;
            }
            clean = true;
            break;

            // Right command
            case KEYCODE_D:
            // Check if the value is already non-zero. If so, decrease it
            ROS_INFO("Commanding backward");
            if(accel >= -10) {
                accel--;
            }
            clean = true;
            break;

            // EBS
            case KEYCODE_E:
            ROS_INFO("EBS active. Stopping...");
            accel = 0;
            turn = 0;
            break;
        }

        // Pack message
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.drive.steering_angle = turn;
        drive_msg.drive.acceleration = accel;
        forward.publish(drive_msg);

        clean = false;        

        rate.sleep();
    }
    return 0;

}