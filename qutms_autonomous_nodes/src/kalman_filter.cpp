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


// Implements an Adaptive Kalman Filter to estimate the state of a platform
// The state is [x y yaw x_vel y_vel yaw_vel]

using namespace std;

class Adaptive_Kalman_Filter {
    // KF matrices 
    Eigen::MatrixXd A, H, P, Q, R, C; // Process matrices and covariances

    Eigen::VectorXd xk;
    Eigen::Vector3d gl, w_en, w_ie, p_imu, v; // State vector

    public:
    // Constructor
    Adaptive_Kalman_Filter();

    // KF functions to do work
    void state_predict(void);
    void state_update(void);
    void cov_update(int);

    void imu_meas(double, double, double, double);
    void gps_meas(double, double);
    void state_pub(void);
    void pos_ins_prop(void);
    void quat_2_eu(double&, double&, double&, double, double, double, double);

    Eigen::Matrix3d euler_to_dcm(double, double, double);

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
    double latm, lngm, psim, lat_dotm, lng_dotm, psi_dotm, h_dot;

    // KF measurement matrix
    Eigen::VectorXd yk, ek;
    Eigen::MatrixXd K;

    // Some other useful variables
    tf2::Quaternion quaternion;
    boost::array<double, 36UL> pose_cov;
    boost::array<double, 36UL> vel_cov;
    double g = 9.79;
    double sigma = 7.292115E-5;
    double R0 = 6378137; 
    vector<double> att;
    Eigen::Matrix3d DCM;
    ofstream outfile;
};

Adaptive_Kalman_Filter::Adaptive_Kalman_Filter() : A(6,6), H(6,6), P(6,6), Q(6,6), R(6,6), C(6,6), xk(6,1) {
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
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 1;

    R = 3*Eigen::MatrixXd::Identity(6,6);

    Q << 1.3479, 0, 0, 0, 0, 0,
        0, 1.3479, 0, 0, 0, 0,
        0, 0, 2*M_PI/180, 0, 0, 0,
        0, 0, 0, 0.1214, 0, 0,
        0, 0, 0, 0, 0.1214, 0,
        0, 0, 0, 0, 0, 2*M_PI/180;

    C = Eigen::MatrixXd::Ones(6,6);

    xk << 49.0086, 8.3981, 2.6006, 0.6984, -0.0183, -0.0062;

    gl << 0, 0, g;
    w_en << 0, 0, 0;
    w_ie << sigma*cos(xk[0]*M_PI/180), 0, -sigma*sin(xk[0]*M_PI/180);
    p_imu << 49.0086448, 8.3981040, 112.9905930;
    v << 0.6984, -0.0183, -0.0021;    
    // Initialise the sizes of the other important vectors
    yk.resize(6,1);

    // Publishers
    state_pose_pub = na.advertise<geometry_msgs::PoseWithCovariance>("/qev2_state_pose", 10);
    state_vel_pub = na.advertise<geometry_msgs::TwistWithCovariance>("/qev2_state_vel", 10);

    // Subscribers
    imu_sub = na.subscribe("/qev2/imu/data", 10, &Adaptive_Kalman_Filter::imu_callback, this);
    gps_sub = na.subscribe("/qev2/gps/data", 10, &Adaptive_Kalman_Filter::gps_callback, this);  

    // Open a text file
    outfile.open("kf_results.txt");  

}

void Adaptive_Kalman_Filter::state_predict(void) {
    // Predict the next state
    xk = A*xk; 

    // Predict the next covariance
    P = A*P*A.transpose() + Q;

    // DEBUG
    cout << "The new state is: " << xk << endl;
}

void Adaptive_Kalman_Filter::state_update(void) {
    // Get the Kalman Gain
    Eigen::MatrixXd residmat = H*P*H.transpose() + R;
    K = P*H.transpose()*residmat.inverse();

    // DEBUG
    cout << "Calculated Kalman Gain as: " << K << endl;

    // State update, assuming we have measurements
    yk << latm, lngm, psim, lat_dotm, lng_dotm, psi_dotm;

    // DEBUG
    // cout << "Got measurements: " << yk << endl;
    xk = xk + K*(yk - H*xk);

    // DEBUG
    cout << "The updated state is: " << xk << endl;

    // Covariance update
    P = (Eigen::MatrixXd::Identity(6,6) - K*H)*P;

    // Get the error matrix
    ek = xk - yk;
}

void Adaptive_Kalman_Filter::cov_update(int k) {
    // Update the state and measurement covariance matrices
    // Innovation covariance matrix
    Eigen::MatrixXd Ck;
    // Sanity check - the error matrix exists
    if (ek.rows() != 0) {
        Ck = ((k-1)/k)*C + (1/k)*ek*ek.transpose();
    } else {
        ROS_ERROR("Error matrix size is 0, defaulting to ones");
        ek = Eigen::VectorXd::Ones(6,1);
        Ck = ((k-1)/k)*C + (1/k)*ek*ek.transpose();
    }

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
    Eigen::Vector3d fbc(1,3), fnc;
    fbc << lat_dotm, lng_dotm, h_dot;
    fnc = DCM*fbc;

    // Propagate velocity
    Eigen::VectorXd v_dot = fnc - v.cross(w_ie);
    v = v + v_dot*0.1; 

    // Get velocity values in lat and long
    double Ldot = v[0]/(R0 + p_imu[2]);
    double ldot = v[1]/(cos(p_imu[0]*M_PI/180)*(R0 + p_imu[2]));
    double hdot = -v[2];

    // Update position
    p_imu[0] = p_imu[0] + Ldot*0.1;
    p_imu[1] = p_imu[1] + ldot*0.1;
    p_imu[2] = p_imu[2] + hdot*0.1;

    // Measurement update
    latm = p_imu[0];
    lngm = p_imu[1];

}

void Adaptive_Kalman_Filter::imu_meas(double psii, double lat_doti, double lng_doti, double psi_doti) {
    // Update measurements from an IMU
    psim = psii;
    lat_dotm = lat_doti;
    lng_dotm = lng_doti;
    psi_dotm = psi_doti;

}

void Adaptive_Kalman_Filter::gps_meas(double lati, double lngi) {
    // Update measurements from a GPS
    latm = lati;
    lngm = lngi; 
}

void Adaptive_Kalman_Filter::quat_2_eu(double& r, double& p, double& ya, double x, double y, double z, double w) {
    // Convert a quaternion (x, y, z, w) represenation to an rpy representation
    r = atan2((2*(y*z + x*w)),(x*x + y*y - z*z - w*w));
    p = asin(-2*(y*w - x*z));
    ya = atan2((2*(z*w + x*y)),(x*x - y*y - z*z + w*w));

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

Eigen::Matrix3d Adaptive_Kalman_Filter::euler_to_dcm(double r, double p, double y) {
    // Transform an RPY representation to a DCM
    Eigen::Matrix3d dcm;

    double d2r = M_PI/180;

    dcm << cos(p*d2r)*cos(y*d2r), -cos(r*d2r)*sin(y*d2r)+sin(r*d2r)*sin(p*d2r)*cos(y*d2r), sin(r*d2r)*sin(y*d2r)+cos(r*d2r)*sin(p*d2r)*cos(y*d2r),
            cos(p*d2r)*sin(y*d2r), cos(r*d2r)*cos(y*d2r)+sin(r*d2r)*sin(p*d2r)*sin(y*d2r), -sin(r*d2r)*cos(y*d2r)+cos(r*d2r)*sin(p*d2r)*sin(y*d2r),
            -sin(p*d2r), sin(r*d2r)*cos(p*d2r), cos(r*d2r)*cos(p*d2r);
    
    return dcm;

}

void Adaptive_Kalman_Filter::info_write(void) {
    // Write out any important information to a text file or possibly a MAT file
    // Check if the file is open. If so, write some data
    if(outfile.is_open()) {
        // Get the updated state and Kalman Gain matrix
        outfile << xk << "\n";
        outfile << K << "\n";
    } else {
        ROS_ERROR("File not opened");
    }

}

void Adaptive_Kalman_Filter::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // Grab the required IMU data
    double x = imu_msg->orientation.x;
    double y = imu_msg->orientation.y;
    double z = imu_msg->orientation.z;
    double w = imu_msg->orientation.w;
    double lat_dot = imu_msg->linear_acceleration.x;
    double lng_dot = imu_msg->linear_acceleration.y;
    double psi_dot = imu_msg->angular_velocity.z;

    // Convert quaternion to rpy
    double phi, theta, psi;
    quat_2_eu(phi, theta, psi, x, y, z, w);

    // Update
    imu_meas(psi, lat_dot, lng_dot, psi_dot);

    // Get a DCM
    DCM = euler_to_dcm(phi, theta, psi);
}

void Adaptive_Kalman_Filter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
    // Get the required GPS data
    double lat = gps_msg->latitude;
    double lng = gps_msg->longitude;
    double h = gps_msg->altitude;

    // Update the p_imu variable
    p_imu << lat, lng, h;

    // Update
    gps_meas(lat, lng);
}

int main(int argc, char **argv) {
    // Init node
    ros::init(argc, argv, "QEV2_AKF");

    // Instantiate class
    Adaptive_Kalman_Filter AKF;

    // Run as fast as the quickest subscriber
    ros::Rate rate(1);

    // Get a counter
    int f = 1;
    while(ros::ok()) {

        // State prediction
        AKF.state_predict();

        // Process any waiting callbacks
        ros::spinOnce();

        // State update
        AKF.state_update();

        // Update our covariances
        //AKF.cov_update(f);

        // Publish the state
        AKF.state_pub();

        // Increment
        f++;

        // Sleep
        rate.sleep();
    }
}