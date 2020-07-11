#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h>
#include <signal.h>
#include <Eigen/Dense>
#include <boost/array.hpp>
#include <iostream>
#include <fstream>
#include <random>


// Implements an Adaptive Kalman Filter to estimate the state of a platform
// The state is [x y yaw x_vel y_vel yaw_vel]

using namespace std;

ofstream outfile;
char dir[300];
char dira[300];
char dirb[300];
char dirc[300];

class Adaptive_Kalman_Filter {
    // KF matrices 
    Eigen::MatrixXd A, H, P, Pa, Q, R, C; // Process matrices and covariances

    Eigen::VectorXd xk, xka; // State vector
    Eigen::Vector3d gl, w_en, w_ie, p_imu, v; 

    public:
    // Constructor
    Adaptive_Kalman_Filter();

    // KF functions to do work
    void state_predict(void);
    void state_update(void);
    void cov_update(int);

    void state_pub(void);
    void pos_ins_prop(void);
    void quat_2_eu(double, double, double, double);
    void info_process(int jj);

    void euler_to_dcm(double, double, double);

    void info_write(void);

    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);

    private:
    // ROS declarations
    ros::NodeHandle na;
    ros::Publisher state_pose_pub;
    ros::Publisher state_vel_pub;
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;

    geometry_msgs::PoseWithCovariance state_pose_msg;
    geometry_msgs::TwistWithCovariance state_vel_msg;

    // Measurement variables
    double latm, lngm, psim, lat_dotm, lng_dotm, psi_dotm, h_dot, phi, theta, psi;

    // KF measurement matrix
    Eigen::VectorXd yk, ek;
    Eigen::MatrixXd K;

    // Some other useful variables
    tf2::Quaternion quaternion;
    boost::array<double, 36UL> pose_cov;
    boost::array<double, 36UL> vel_cov;
    double g = 9.7754;
    double sigma = 7.292115E-5;
    double R0 = 6378137; 
    vector<double> att;
    Eigen::Matrix3d DCM;
    double d2r = M_PI/180;

    vector<double> gps_data, imu_accel_data, imu_rate_data, att_data;

};

void SigIntHandler(int sig) {
    // Close the opened text file, and shutdown
    ROS_INFO("Closing opened data file...");
    outfile.close();
    ROS_ERROR("File closed, shutting down");
    ros::shutdown();
}

Adaptive_Kalman_Filter::Adaptive_Kalman_Filter() : A(6,6), H(6,6), P(6,6), Pa(6,6), Q(6,6), R(6,6), C(6,6), xk(6,1), xka(6,1) {

    // Setup all necessary variables
    A <<  1, 0, 0, 0.1, 0, 0,
        0, 1, 0, 0, 0.1, 0,
        0, 0, 1, 0, 0, 0.1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    P = 400*Eigen::MatrixXd::Identity(6,6);

    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 1;

    R = 3*Eigen::MatrixXd::Identity(6,6);

    Q << 1.3479, 0, 0, 0, 0, 0,
        0, 1.3479, 0, 0, 0, 0,
        0, 0, 2*d2r, 0, 0, 0,
        0, 0, 0, 0.1214, 0, 0,
        0, 0, 0, 0, 0.1214, 0,
        0, 0, 0, 0, 0, 2*d2r;

    C = Eigen::MatrixXd::Ones(6,6);

    // Initialise the state and other vectors
    xk << 49.0086*d2r, 8.3981*d2r, 2.6006, 0.6984, -0.0183, -0.0062;

    gl(0) = -0.0168; 
    gl(1) = 0; 
    gl(2) = g;
    
    w_en(0) = 0; 
    w_en(1) = 0; 
    w_en(2) = 0;
    
    w_ie(0) = sigma*cos(xk(0)); 
    w_ie(1) = 0; 
    w_ie(2) = -sigma*sin(xk(0));
    
    p_imu(0) = 49.0086448*d2r; 
    p_imu(1) = 8.3981040*d2r; 
    p_imu(2) = 112.9905930;

    v(0) = 0.6984; 
    v(1) = -0.0183; 
    v(2) = -0.0021;    
    
    // Initialise the sizes of the other important vectors
    yk.resize(6,1);

    // Publishers
    state_pose_pub = na.advertise<geometry_msgs::PoseWithCovariance>("/qev2_state_pose", 10);
    state_vel_pub = na.advertise<geometry_msgs::TwistWithCovariance>("/qev2_state_vel", 10);

    // Subscribers
    imu_sub = na.subscribe("/KITTI_imu", 10, &Adaptive_Kalman_Filter::imu_callback, this);
    gps_sub = na.subscribe("/KITTI_gps", 10, &Adaptive_Kalman_Filter::gps_callback, this);  

    // Get the custom shutdown handler
    signal(SIGINT, SigIntHandler);

    // Setup for rand generator
    const double mean = 0;
    const double sigma = 0.5;
    auto dist = std::bind(std::normal_distribution<double>{mean, sigma},std::mt19937(std::random_device{}()));

    // Get the GPS file
    sprintf(dir, "/home/nick-hp-ubuntu/Documents/Text Files/KITTI_gps.txt");

    // Process the text file
    string line;
    ifstream infile (dir);
    if(infile.is_open()) {
        while(getline(infile, line)) {
            double num10, num11, num12;
            infile >> num10 >> num11 >> num12;
            // cout << "Got " << num1 << " " << num2 << " " << num3 << endl;
            gps_data.push_back((num10 + dist())*d2r);
            gps_data.push_back((num11 + dist())*d2r);
            gps_data.push_back((num12 + dist())*d2r);
        }
    }
    infile.close();

        // Get 2 directories to a text file
    sprintf(dira, "/home/nick-hp-ubuntu/Documents/Text Files/KITTI_accel_xyz.txt");
    sprintf(dirb, "/home/nick-hp-ubuntu/Documents/Text Files/KITTI_brate_xyz.txt");
    sprintf(dirc, "/home/nick-hp-ubuntu/Documents/Text Files/KITTI_att.txt");

    // Process the text file for acceleration data
    string aline;
    ifstream ainfile (dira);
    if(ainfile.is_open()) {
        while(getline(ainfile, aline)) {
            double num1, num2, num3;
            ainfile >> num1 >> num2 >> num3;
            // cout << "Got " << num1 << " " << num2 << " " << num3 << endl;
            imu_accel_data.push_back(num1 + dist());
            imu_accel_data.push_back(num2 + dist());
            imu_accel_data.push_back(num3 + dist());
        }
    }
    ainfile.close();

    // Open up and process the next file
    string rline;
    ifstream rinfile (dirb);
    if(rinfile.is_open()) {
        while(getline(rinfile, rline)) {
            double num4, num5, num6;
            rinfile >> num4 >> num5 >> num6;
            // cout << "Got " << num4 << " " << num5 << " " << num6 << endl;
            imu_rate_data.push_back(num4 + dist());
            imu_rate_data.push_back(num5 + dist());
            imu_rate_data.push_back(num6 + dist());
        }
    }
    rinfile.close();

    // Process attitude data
    string tline;
    ifstream tinfile (dirc);
    if(tinfile.is_open()) {
        while(getline(tinfile, tline)) {
            double num7, num8, num9;
            tinfile >> num7 >> num8 >> num9;
            // cout << "Got " << num7 << " " << num8 << " " << num9 << endl;
            att_data.push_back(num7 + dist());
            att_data.push_back(num8 + dist());
            att_data.push_back(num9 + dist());
        }
    }
    tinfile.close();

    // Open a text file
    outfile.open("/home/nick-hp-ubuntu/catkin_ws/src/qutms_autonomous_nodes/kf_results.txt");

    // Time to get to here: 0.013s

}

void Adaptive_Kalman_Filter::state_predict(void) {
    // Predict the next state
    xka = A*xk; 

    // Predict the next covariance
    P = A*P*A.transpose() + Q;

    // DEBUG
    // cout << "The new state is: " << xk << endl;
}

void Adaptive_Kalman_Filter::state_update(void) {
    // Get the Kalman Gain
    Eigen::MatrixXd residmat = H*P*H.transpose() + R;
    K = P*H.transpose()*residmat.inverse();

    // DEBUG
    // cout << "Calculated Kalman Gain as: " << K << endl;

    // State update, assuming we have measurements
    // ROS_INFO("Passing %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f to measurement vector", latm, lngm, psim, lat_dotm, lng_dotm, psi_dotm);
    yk << latm, lngm, psim, lat_dotm, lng_dotm, psi_dotm;

    // DEBUG
    // cout << "Got measurements: " << yk << endl;
    xk = xka + K*(H*yk - H*xka);

    // DEBUG
    cout << "The updated state is: " << xk << endl;

    // Covariance update
    P = (Eigen::MatrixXd::Identity(6,6) - K*H)*P;

    // Get the error matrix
    ek = H*yk - H*xk;
}

void Adaptive_Kalman_Filter::cov_update(int k) {
    // Update the state and measurement covariance matrices
    // Innovation covariance matrix
    Eigen::MatrixXd Ck;

    // Calculate C using the error matrix
    Ck = ((k-1)/k)*C + (1/k)*ek*ek.transpose();

    // Update measurement covariance
    R = Ck + H*P*H.transpose();

    // Update state covariance
    Q = K*Ck*K.transpose();

    // Save the new innovation covariance
    C = Ck;
}

void Adaptive_Kalman_Filter::pos_ins_prop(void) {
    // Propagates acceleration measurements to estimate position

    // Transform acceleration values
    Eigen::Vector3d fbc;
    fbc.resize(3,1);
    fbc << lat_dotm, lng_dotm, h_dot;
    Eigen::VectorXd fnc = DCM*fbc;
    // cout << fnc << endl;

    // Propagate velocity
    Eigen::Vector3d w_ie_t = 2*w_ie.transpose() + w_en.transpose();
    Eigen::VectorXd v_dot = fnc - w_ie_t.cross(v) + gl;
    v = v + v_dot*0.1; 

    // cout << "Velocity and acceleration is: " << v << "\n" << v_dot << endl;
    double vx = v(0);
    double vy = v(1);
    double vz = v(2);
    double xt = p_imu(0);
    double yt = p_imu(1);
    double zt = p_imu(2);

    // Get velocity values in lat and long
    double Ldot = vx/(R0 + zt);
    double ldot = vy*(1/(cos(xt))/(R0 + zt));
    double hdot = -vz;

    // Update position
    latm = xt + Ldot*0.1;
    lngm = yt + ldot*0.1;
    double hm = zt + hdot*0.1;

    // Measurement update
    p_imu << latm, lngm, hm;

    // cout << "The new IMU position is: " << p_imu << endl;

}

void Adaptive_Kalman_Filter::quat_2_eu(double x, double y, double z, double w) {
    // Convert a quaternion (x, y, z, w) represenation to an rpy representation
    psi = atan2((2*(y*z + x*w)),(x*x + y*y - z*z - w*w));
    theta = asin(-2*(y*w - x*z));
    phi = atan2((2*(z*w + x*y)),(x*x - y*y - z*z + w*w));

}

void Adaptive_Kalman_Filter::state_pub(void) {
    // Publishes the estimated state from the KF process with covariances attached

    // Get the state pose
    state_pose_msg.pose.position.x = yk[0];
    state_pose_msg.pose.position.y = yk[1];
    quaternion.setRPY(0, 0, yk[2]);
    state_pose_msg.pose.orientation = tf2::toMsg(quaternion);

    // Get the state pose covariance
    pose_cov[0] = P(0,0);
    pose_cov[7] = P(1,1);
    pose_cov[35] = P(2,2);
    state_pose_msg.covariance = pose_cov;

    // Manage the velocity covariances
    state_vel_msg.twist.linear.x = yk[3];
    state_vel_msg.twist.linear.y = yk[4];
    state_vel_msg.twist.angular.z = yk[5];

    // Velocity covariance
    vel_cov[0] = P(3,3);
    vel_cov[7] = P(4,4);
    vel_cov[35] = P(5,5);

    // Publish
    state_pose_pub.publish(state_pose_msg);
    state_vel_pub.publish(state_vel_msg);
}

void Adaptive_Kalman_Filter::info_process(int jj) {
    // Dummy function to imitate subscribers receiving information
    // Grabs information from vectors and passes it to sensor information vector

    // Assign IMU data
    lat_dotm = imu_accel_data[jj];
    lng_dotm = imu_accel_data[jj+1];
    h_dot = imu_accel_data[jj+2];
    psi_dotm = imu_rate_data[jj+2];
    psim = att_data[jj+2];

    // Get a DCM
    euler_to_dcm(att_data[jj], att_data[jj+1], att_data[jj+2]);

    // Check once 10 iterations has been done, and assign the GPS data
    pos_ins_prop(); // Propagate position with INS mechanisation
    int iteration = jj % 10;
    if (!iteration) {
        latm = gps_data[jj];
        lngm = gps_data[jj+1];
        p_imu << latm, lngm, gps_data[jj+2];
    }
}

void Adaptive_Kalman_Filter::euler_to_dcm(double r, double p, double y) {
    // Transform an RPY representation to a DCM

    DCM << cos(p)*cos(y), -cos(r)*sin(y)+sin(r)*sin(p)*cos(y), sin(r)*sin(y)+cos(r)*sin(p)*cos(y),
            cos(p)*sin(y), cos(r)*cos(y)+sin(r)*sin(p)*sin(y), -sin(r)*cos(y)+cos(r)*sin(p)*sin(y),
            -sin(p), sin(r)*cos(p), cos(r)*cos(p);

    // cout << "Calculated DCM " << DCM << endl;

}

void Adaptive_Kalman_Filter::info_write(void) {
    // Write out any important information to a text file or possibly a MAT file
    // Check if the file is open. If so, write some data
    if(outfile.is_open()) {
        // Get the updated state and Kalman Gain matrix
        outfile << xk << "\n";
        // outfile << K << "\n";
    } else {
        ROS_ERROR("File not opened, unable to write information");
    }

}

void Adaptive_Kalman_Filter::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // Grab the required IMU data
    double x = imu_msg->orientation.x;
    double y = imu_msg->orientation.y;
    double z = imu_msg->orientation.z;
    double w = imu_msg->orientation.w;
    Adaptive_Kalman_Filter::lat_dotm = imu_msg->linear_acceleration.x;
    Adaptive_Kalman_Filter::lng_dotm = imu_msg->linear_acceleration.y;
    Adaptive_Kalman_Filter::h_dot = imu_msg->linear_acceleration.z;
    Adaptive_Kalman_Filter::psi_dotm = imu_msg->angular_velocity.z;
    // double lat_dot = imu_msg->linear_acceleration.x;
    // double lng_dot = imu_msg->linear_acceleration.y;
    // double psi_dot = imu_msg->angular_velocity.z;

    // Convert quaternion to rpy
    quat_2_eu(x, y, z, w);

    // Get a DCM
    euler_to_dcm(phi, theta, psi);

    // Propagate position with the IMU data
    pos_ins_prop();
}

void Adaptive_Kalman_Filter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
    // Get the required GPS data
    double lat = gps_msg->latitude;
    double lng = gps_msg->longitude;
    double h = gps_msg->altitude;

    // Update
    Adaptive_Kalman_Filter::latm = lat;
    Adaptive_Kalman_Filter::lngm = lng;
    
    // Update the p_imu variable
    p_imu << lat, lng, h;
}

int main(int argc, char **argv) {
    // Init node
    ros::init(argc, argv, "QEV2_AKF", ros::init_options::NoSigintHandler);

    // Instantiate class
    Adaptive_Kalman_Filter AKF;

    // Run as fast as the quickest publisher
    ros::Rate rate(1);

    // Get a counter
    int f = 1;
    int n = 3;
    while((ros::ok()) && (f != 999)) {

        // State prediction
        AKF.state_predict();

        // Process any waiting callbacks
        // ros::spinOnce();
        AKF.info_process(n);

        // State update
        AKF.state_update();

        // Update the covariances
        // AKF.cov_update(f);

        // Publish the state
        AKF.state_pub();

        // Increment
        f++;
        n = n + 3;

        // Save the required information
        AKF.info_write();

        // Sleep
        rate.sleep();
    }
}