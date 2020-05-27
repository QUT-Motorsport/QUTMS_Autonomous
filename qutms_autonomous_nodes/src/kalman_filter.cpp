#include <ros/ros.h>
#include <std_msgs/Float64>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <math.h>
#include <Eigen/Dense>

// Implements an Adaptive Kalman Filter to estimate position
// The state is [x y yaw x_vel y_vel yaw_vel]

using namespace std;

class Adaptive_Kalman_Filter {
    public:
    // Constructor
    Adaptive_Kalman_Filter();

    private:
    // ROS variables
    ros::NodeHandle na;
    ros::Publisher state_pos_pub;
    ros::Publisher state_vel_pub;
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;

    geometry_msgs::PoseWithCovariance qev2_pose_msg;
    geometry_msgs::TwistWithCovariance qev2_twist_msg;

    void predict_step(Eigen::VectorXd& xk_hatp, Eigen::MatrixXd& Pku, Eigen::MatrixXd& Qa, int k);

    void update_step(Eigen::VectorXd& xk_hat, Eigen::MatrixXd& Pk, Eigen::MatrixXd Ra, Eigen::MatrixXd Ha, Eigen::VectorXd yk, int k);

    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);

    // Matrices for AKF
    Eigen::MatrixXd A;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::Matrix<double, 6, 6>::Identity() R;
    Eigen::MatrixXd P;

    Eigen::VectorXd xk;

    double dt = 0.1;

    double lat;
    double lng;
    double psi;
    double lat_dot;
    double lng_dot;
    double psi_dot;

};

Adaptive_Kalman_Filter::Adaptive_Kalman_Filter() {
    // Setup for KF process
    A << 1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, dt, 0, 0, 0,
        0, 0, 0, dt, 0, 0,
        0, 0, 0, 0, dt, 0,
        0, 0, 0, 0, 0, 1;

    R = 3*R;

    Q << 0.5, 0, 0, 0, 0, 0
        0, 0.5, 0, 0, 0, 0
        0, 0, M_PI_2/180, 0, 0, 0
        0, 0, 0, 0.5, 0, 0
        0, 0, 0, 0, 0.5, 0
        0, 0, 0, 0, 0, 2*M_PI_2/180;

    xk << lat, lng, psi, lat_dot, lng_dot, psi_dot;

    // Publishers
    state_pos_pub = na.advertise<geometry_msgs::PoseWithCovariance>("/qev2_state_pose", 10);
    state_vel_pub = na.advertise<geometry_msgs::TwistWithCovariance>("/qev2_state_vel", 10);

    // Subscribers
    imu_sub = na.subscribe("/qev2/imu/data", 10, &Adaptive_Kalman_Filter::imu_callback, this);
    gps_sub = na.subscribe("/qev2/gps/data", 10, &Adaptive_Kalman_Filter::gps_callback, this); 

}

void Adaptive_Kalman_Filter::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // Parse an incoming message

}

void Adaptive_Kalman_Filter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {

}

void Adaptive_Kalman_Filter::predict_step(Eigen::VectorXd& xk_hatp, Eigen::MatrixXd& Pku, Eigen::MatrixXd& Qa, int k) {
    // Predict the platforms' next state at iteration k

    // Predict the new state
    xk_hatp = A*xk_hatp;

    // Predict the covariance
    Pku = A*Pku*A.transpose() + Q;
}

void Adaptive_Kalman_Filter::update_step(Eigen::VectorXd& xk_hat, Eigen::MatrixXd& Pk, Eigen::MatrixXd Ra, Eigen::MatrixXd Ha, Eigen::VectorXd yk, int k) {
    // Update the state and covariance matrices

    // Kalman Gain
    Eigen::MatrixXd Y = Ha*Pk*Ha.transpose() + Ra;
    Eigen::MatrixXd K = Pk*Ha.transpose()*Y.inverse();

    // State update
    xk_hat = xk_hat + K*(yk - Ha*xk_hat);

    // Covariance update
    Pk = (MatrixXd::Identity(6,6) -K*Ha)*Pk;

    // Error matrix
    e = yk - Ha*xk_hat;
}

int main(int argc, char **argv) {
    // Init
    ros::init(argc, argv, "QEV2_AKF");

    // Run and spin
    Adaptive_Kalman_Filter AKF;

    ros::spin();
}