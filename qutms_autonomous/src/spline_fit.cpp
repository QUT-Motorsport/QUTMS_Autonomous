#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <math.h>
#include <Eigen/Dense>

using namespace std;
Eigen::Vector2d spline_eval(vector<double> xps, vector<double> yps, double u) {
    // Requires four points to return a spline fitted to each point
    int T = 0; // Catmull-Rom spline
    double s = (1-T)/2;

    // Cardinal matrix
    Eigen::Matrix4d mc;
    mc << -s, 2-s, s-2, s,
        2*s, s-3, 3-(2*s), -s,
        -s, 0.0, s, 0.0,
        0.0, 1.0, 0.0, 0.0; 
    //cout << mc << endl;

    Eigen::Vector4d Ghx, Ghy;
    Ghx << xps[0], xps[1], xps[2], xps[3];
    Ghy << yps[0], yps[1], yps[2], yps[3];
    //cout << Ghx << "\n" << Ghy << endl;

    Eigen::Vector4d U;
    U << u*u*u, u*u, u, 1;
    //cout << U << endl;

    // Make the points
    double xt = U.transpose()*mc*Ghx;
    double yt = U.transpose()*mc*Ghy;

    cout << xt << "\n" << yt << endl;

    Eigen::Vector2d p_out;
    p_out << xt, yt;

    return p_out;

}

void pose_callback(const geometry_msgs::PoseArrayConstPtr& pose_msg) {
    // Handles the positions input and returns curve points
    geometry_msgs::Pose landmark_poses;
    vector<double> xvec;
    vector<double> yvec;

    // We expect 4 points at a time
    ROS_INFO("Processing %lu points", pose_msg->poses.size());
    for (int i = 0; i <= pose_msg->poses.size()-1; i++) {
        landmark_poses = pose_msg->poses[i];
        xvec.push_back(landmark_poses.position.x);
        yvec.push_back(landmark_poses.position.y);
        ROS_INFO("Grabbed point (%0.2f, %0.2f)", xvec[i], yvec[i]);
    }

    // Make new vectors to separate the points out
    vector<double> xbl, ybl, xye, yye;
    for (int i = 0; i <= 3; i++) {
        xbl.push_back(xvec[2*i]);
        ybl.push_back(yvec[2*i]);
        xye.push_back(xvec[2*i + 1]);
        yye.push_back(yvec[2*i + 1]);
    }

    // Get spline points
    Eigen::Vector2d sbp, syp;

    sbp = spline_eval(xbl, ybl, 0.5);
    //ROS_INFO("Got %ld spline points", s_points.size());

    syp = spline_eval(xye, yye, 0.5);
    //ROS_INFO("Got %ld spline points", s_points.size());

}

int main(int argc, char **argv) {
    // Node init
    ros::init(argc, argv, "Curve_fitter");

    // Node Handle
    ros::NodeHandle nc;

    // Subscriber
    ros::Subscriber pose_sub = nc.subscribe("qev2/landmark_poses", 10, pose_callback);

    // Spin
    ros::spin();
}