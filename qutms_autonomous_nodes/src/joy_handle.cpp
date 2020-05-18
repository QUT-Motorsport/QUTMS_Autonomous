#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>
#include <map>
#include <string>
#include <math.h>

class Joy_to_Twist {
    public:
    // Constructor here
    Joy_to_Twist();

    geometry_msgs::Twist get_twist(float f_b, float l_r) {
    // Handle the joystick value conversions
    geometry_msgs::Twist twist_temp;

    // Determine heading
    double opp = M_PI_2*f_b;
    double adj = M_PI_2*l_r;
    double yaw = atan2(opp, adj);

    // Make sure we always get a 0 yaw for 0 x and y
    if ((f_b == 0) && (l_r == 0)) {
        yaw = 0.0;
    }
    
    // If we arent in the manual state, these should be 0
    if(nj.getParam("AS_State", state)) {
        if(state == "AS_Manual") {
            twist_temp.linear.x = f_b*vel_x;
            twist_temp.linear.y = l_r*vel_y;
            twist_temp.angular.z = yaw;
        } else {
            twist_temp.linear.x = 0.0;
            twist_temp.linear.y = 0.0;
            twist_temp.angular.z = 0.0;
        }
    }
        
    return twist_temp;
    }
    
    // Private variables
    private:
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void handle_param(void);
    // ROS private parameters
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::NodeHandle nj;

    geometry_msgs::Twist qev2_twist_msg;

    // Some handy variables
    double vel_x;
    double vel_y;
    double turn;

    std::string state;
    
};

Joy_to_Twist::Joy_to_Twist() {
    // Handle parameters for XYZ scaling
    ROS_INFO("Setting parameter values");
    // Get parameters
    nj.param("vel_base_x", vel_x, 2.0);
    nj.param("vel_base_y", vel_y, 2.0);
    nj.param("turn_rate", turn, 1.0);
    ROS_INFO("Parameters set");

    // Publishers
    vel_pub = nj.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Set subscriber
    joy_sub = nj.subscribe("/joy", 10, &Joy_to_Twist::joy_callback, this);
}

void Joy_to_Twist::joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        // Handle the Joystick message here
        // Do conversion from Joy to Twist here
        // For sanity
        if(joy_msg->axes.size() != 8) {
            ROS_ERROR("Not enough elements");
        }

        float stick_fb = joy_msg->axes[0];
        float stick_lr = joy_msg->axes[1];
        // ROS_INFO("Got values %0.2f, %0.2f, %0.2f, %0.2f", qw, qx, qy, qz);

        qev2_twist_msg = get_twist(stick_fb, stick_lr);

        vel_pub.publish(qev2_twist_msg);

    }

int main(int argc, char **argv) {
    // Init
    ros::init(argc, argv, "Joy_to_Twist");

    // Run and spin
    Joy_to_Twist joy_twist;

    ros::spin();

}
