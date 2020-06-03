#include <ros/ros.h>
#include <std_msgs/Float64.h>
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

    void predict_step(Eigen::VectorXd xk_hatp, Eigen::MatrixXd Ak, Eigen::MatrixXd Pku, Eigen::MatrixXd Qk);
    void update_step(Eigen::VectorXd xk_hatp, Eigen::MatrixXd Pku, Eigen::VectorXd y, Eigen::MatrixXd Hk, Eigen::MatrixXd R);
    void get_meas(double ltm, double lnm, double pim, double ltdm, double lndm, double pidm);

    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);

    // Matrices for AKF
    Eigen::MatrixXd Aa, H, Q, Pa, R; 

    Eigen::VectorXd xk, yk;

    double dt;

    double lat, lng, psi, lat_dot, lng_dot, psi_dot, latm, lngm, psim, lat_dotm, lng_dotm, psi_dotm;

    void set_filter(Eigen::MatrixXd Aa, Eigen::MatrixXd H, Eigen::MatrixXd Q, Eigen::MatrixXd Pa, Eigen::MatrixXd R);
    void init_state(Eigen::VectorXd xf);

};

Adaptive_Kalman_Filter::Adaptive_Kalman_Filter() {

    // Publishers
    state_pos_pub = na.advertise<geometry_msgs::PoseWithCovariance>("/qev2_state_pose", 10);
    state_vel_pub = na.advertise<geometry_msgs::TwistWithCovariance>("/qev2_state_vel", 10);

    // Subscribers
    imu_sub = na.subscribe("/qev2/imu/data", 10, &Adaptive_Kalman_Filter::imu_callback, this);
    gps_sub = na.subscribe("/qev2/gps/data", 10, &Adaptive_Kalman_Filter::gps_callback, this); 

}

void Adaptive_Kalman_Filter::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // Grab the required IMU data
    Adaptive_Kalman_Filter::psim = imu_msg->orientation.z;
    Adaptive_Kalman_Filter::lat_dotm = imu_msg->linear_acceleration.x;
    Adaptive_Kalman_Filter::lng_dotm = imu_msg->linear_acceleration.y;
    Adaptive_Kalman_Filter::psi_dotm = imu_msg->angular_velocity.z;

}

void Adaptive_Kalman_Filter::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
    // Get the required GPS data
    Adaptive_Kalman_Filter::latm = gps_msg->latitude;
    Adaptive_Kalman_Filter::lngm = gps_msg->longitude;

}

void Adaptive_Kalman_Filter::predict_step(Eigen::VectorXd xk_hatp, Eigen::MatrixXd Ak, Eigen::MatrixXd Pku, Eigen::MatrixXd Qk) {
    // Predict the next state
    this->xk = Ak*xk_hatp; 

    // Predict the next covariance
    this->Pa = Ak*Pku*Ak.transpose() + Qk;
}

void Adaptive_Kalman_Filter::update_step(Eigen::VectorXd xk_hatp, Eigen::MatrixXd Pku, Eigen::VectorXd y, Eigen::MatrixXd Hk, Eigen::MatrixXd R) {
    // Get the Kalman Gain
    Eigen::MatrixXd residmat = Hk*Pku*Hk.transpose() + R;
    Eigen::MatrixXd K = Pku*Hk.transpose()*residmat.inverse();

    // State update
    this->xk = xk_hatp + K*(y - Hk*xk_hatp);

    // Covariance update
    this->Pa = (Eigen::MatrixXd::Identity(6,6) - K*Hk)*Pku;
}

void Adaptive_Kalman_Filter::set_filter(Eigen::MatrixXd As, Eigen::MatrixXd Hs, Eigen::MatrixXd Qs, Eigen::MatrixXd Ps, Eigen::MatrixXd Rs) {
    // Initialize KF matrices
    this->Aa = As;
    this->H = Hs;
    this->Q = Qs;
    this->Pa = Ps;
    this->R = Rs;

}

void Adaptive_Kalman_Filter::init_state(Eigen::VectorXd xf) {
    // Initializes the state 
    this->xk = xf;

}

void Adaptive_Kalman_Filter::get_meas(double ltm, double lnm, double pim, double ltdm, double lndm, double pidm) {
    // Get a measurement model
    Eigen::VectorXd yu;
    yu << ltm, lnm, pim, ltdm, lndm, pidm;

    this->yk = yu;
}

int main(int argc, char **argv) {
    // Init
    ros::init(argc, argv, "QEV2_AKF");

    // Setup
    Adaptive_Kalman_Filter AKF;

    // Setup for KF process
    Eigen::MatrixXd Ai, Hi, Qi;
    Eigen::VectorXd xi;

    Ai <<  1, 0, 0, 0.1, 0, 0,
        0, 1, 0, 0, 0.1, 0,
        0, 0, 1, 0, 0, 0.1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    Eigen::MatrixXd Pi = 400*Eigen::MatrixXd::Identity(6,6);

    Hi << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 1;

    Eigen::MatrixXd Ri = Eigen::MatrixXd::Identity(6,6);

    Qi << 1.3479, 0, 0, 0, 0, 0,
        0, 1.3479, 0, 0, 0, 0,
        0, 0, 2*M_PI/180, 0, 0, 0,
        0, 0, 0, 0.1214, 0, 0,
        0, 0, 0, 0, 0.1214, 0,
        0, 0, 0, 0, 0, 2*M_PI/180;

    xi << 49.0086, 8.3981, 2.6006, 0.6984, -0.0183, -0.0062;

}