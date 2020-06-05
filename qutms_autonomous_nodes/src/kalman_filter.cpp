#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <math.h>
#include <Eigen/Dense>

// Implements an Adaptive Kalman Filter to estimate the state of a platform
// The state is [x y yaw x_vel y_vel yaw_vel]

using namespace std;

class Adaptive_Kalman_Filter {
    // KF matrices 
    Eigen::MatrixXd A, H, P, Q, R, C; // Process matrices and covariances

    Eigen::VectorXd xk; // State vector

    public:
    // Constructor
    Adaptive_Kalman_Filter();

    // KF functions to do work
    void state_predict(void);
    void state_update(void);
    void cov_update(int);

    void imu_meas(double, double, double, double);
    void gps_meas(double, double);

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
    double latm, lngm, psim, lat_dotm, lng_dotm, psi_dotm;

    // KF measurement matrix
    Eigen::VectorXd yk, ek;
    Eigen::MatrixXd K;
};

Adaptive_Kalman_Filter::Adaptive_Kalman_Filter() : A(6,6), H(6,6), P(6,6), Q(6,6), R(6,6), C(6,6), xk(6,1){
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

    // Initialise the sizes of the other important vectors
    yk.resize(6,1);

    // Publishers
    state_pose_pub = na.advertise<geometry_msgs::PoseWithCovariance>("/qev2_state_pose", 10);
    state_vel_pub = na.advertise<geometry_msgs::TwistWithCovariance>("/qev2_state_vel", 10);

    // Subscribers
    imu_sub = na.subscribe("/qev2/imu/data", 10, &Adaptive_Kalman_Filter::imu_callback, this);
    gps_sub = na.subscribe("/qev2/gps/data", 10, &Adaptive_Kalman_Filter::gps_callback, this);     
}

void Adaptive_Kalman_Filter::state_predict(void) {
    // Predict the next state
    xk = A*xk; 

    // Predict the next covariance
    P = A*P*A.transpose() + Q;

    // // DEBUG
    // cout << "The new state is: " << xk << endl;
}

void Adaptive_Kalman_Filter::state_update(void) {
    // Get the Kalman Gain
    Eigen::MatrixXd residmat = H*P*H.transpose() + R;
    Eigen::MatrixXd K = P*H.transpose()*residmat.inverse();

    // State update, assuming we have measurements
    if (yk.rows() != 0) {
        // DEBUG
        cout << "Got measurements: " << yk << endl;
        xk = xk + K*(yk - H*xk);
    } else {
        ROS_ERROR("No measurements retrieved!!");
    }

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

void Adaptive_Kalman_Filter::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // Grab the required IMU data
    double psi = imu_msg->orientation.z;
    double lat_dot = imu_msg->linear_acceleration.x;
    double lng_dot = imu_msg->linear_acceleration.y;
    double psi_dot = imu_msg->angular_velocity.z;

    // Update
    imu_meas(psi, lat_dot, lng_dot, psi_dot);
}

void Adaptive_Kalman_Filter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
    // Get the required GPS data
    double lat = gps_msg->latitude;
    double lng = gps_msg->longitude;

    // Update
    gps_meas(lat, lng);
}

int main(int argc, char **argv) {
    // Init node
    ros::init(argc, argv, "QEV2_AKF");

    // Instantiate class
    Adaptive_Kalman_Filter AKF;

    // Get a counter
    int f = 1;
    while(ros::ok()) {

        // State prediction
        AKF.state_predict();

        // Process any waiting callbacks
        ros::spinOnce();

        // State update
        AKF.state_update();

        // // Update our covariances
        // AKF.cov_update(f);

    }
}