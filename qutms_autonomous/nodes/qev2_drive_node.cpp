#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <time.h>
#include <termios.h>

// Node to send specific messages to the Gazebo node on the QEV2/cmd_vel topic

void set_speed(double vel, std_msgs::Float32& msg) {
    // Takes a double and puts it into the message for publishing
    msg.data = vel;
}

void set_turn(double turn, std_msgs::Float64& msg) {
    // Takes a double and puts into a message for publishing
    msg.data = turn;
}

int getch() {
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt); // Save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON); // Disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt); // Apply new settings

  int c = getchar(); // Read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt); // Restore old settings
  return c;
}

int main(int argc, char **argv) {
    // Initialise
    ros::init(argc, argv, "gazebo_interface_node");

    // Node Handles
    ros::NodeHandle nv;
    ros::NodeHandle nt;

    // Setup messages
    std_msgs::Float32 velr_msg;
    std_msgs::Float64 turn_msg;

    // Publishers
    ros::Publisher gzv_pub = nv.advertise<std_msgs::Float32>("/QEV2/vel_cmd", 5, true);
    ros::Publisher gzt_pub = nt.advertise<std_msgs::Float64>("/QEV2/turn_cmd", 5, true);

    // Rate set
    ros::Rate rate(1);

    // Get the start time of the node
    double start = ros::Time::now().toSec();

    while (ros::ok()) {
        // Set a duration for timing
        ros::Duration d(10.0);
        double dur = d.toSec();
        
        int c = getch(); // Call non-blocking input function
        
        if(c=='r') // Reverse
          set_speed(-10.0, velr_msg);
        if(c=='w') // Forward
          set_speed(10.0, velr_msg);
        if(c=='s') // Stop
          set_speed(0.0, velr_msg);

        if(c=='p') // Vroooom
          set_speed(1000, velr_msg);

        if(c=='q') // Reset  front wheels
          set_turn(0.0, turn_msg);
        if(c=='a')
          set_turn(3.0, turn_msg);
        if(c=='d')
          set_turn(-3.0, turn_msg);
 
        /*
        // Set the initial speed and turn to target
        set_speed(10.0, velr_msg);
        set_turn(2.0, turn_msg);

        // Wait 4 seconds and change to a different speed
        double diff = ros::Time::now().toSec() - start;
        if (diff >= dur) {
            // Slow down and turn the other way
            set_speed(5.0, velr_msg);
            set_turn(-2.0, turn_msg);
        }
        */
        
        // Publish, and sleep
        gzv_pub.publish(velr_msg);
        gzt_pub.publish(turn_msg);

        rate.sleep();
    }
}
