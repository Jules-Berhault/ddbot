#include "ros/ros.h"
#include "tf/tf.h"
#include <eigen3/Eigen/Dense>
#include "stdlib.h"
#include "cmath"

#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

void kalman_predict(Eigen::Vector3d& x1, Eigen::Matrix3d& Gx1, Eigen::Vector3d& xup, Eigen::Matrix3d& Gup, Eigen::Vector2d& u, Eigen::Matrix3d& Galpha, Eigen::Matrix3d& A,  Eigen::MatrixXd& B) {
    Gx1 = A * Gup * A.transpose() + Galpha;
    x1 = A * xup + B * u;
}

void kalman_correct(Eigen::Vector3d&xup, Eigen::Matrix3d& Gup, Eigen::Vector3d& x0, Eigen::Matrix3d& Gx0, Eigen::Vector3d& y, Eigen::Matrix3d& Gbeta, Eigen::Matrix3d& C) {
    Eigen::MatrixXd S = C * Gx0 * C.transpose() + Gbeta;
    Eigen::MatrixXd K = Gx0 * C.transpose() * S.inverse();
    Eigen::VectorXd ytilde = y - C * x0;
    Gup = (Eigen::MatrixXd::Identity(x0.size(), x0.size()) - K * C) * Gx0;
    xup = x0 + K * ytilde;
}

void kalman(Eigen::Vector3d& x0, Eigen::Matrix3d& Gx0, Eigen::Vector2d& u, Eigen::Matrix3d& Galpha, Eigen::Matrix3d& A, Eigen::Vector3d& y, Eigen::Matrix3d& Gbeta, Eigen::Matrix3d& C, Eigen::MatrixXd& B) {
    Eigen::Vector3d xup;
    Eigen::Matrix3d Gup;
    kalman_correct(xup, Gup, x0, Gx0, y, Gbeta, C);
    kalman_predict(x0, Gx0, xup, Gup, u, Galpha, A, B);
}

// Vectors of the system
double h = 1/10;
double theta = 0.0;
Eigen::Vector3d X = {0.0, 0.0};
Eigen::Vector3d Y = {0.0, 0.0};
Eigen::Vector2d u = {0.0, 0.0};

void yaw_Callback(const std_msgs::Float64::ConstPtr& msg){
    theta = msg->data;
}

void cartesian_Callback(const geometry_msgs::PointStamped::ConstPtr& msg){
    Y[0] = msg->point.x;
    Y[1] = msg->point.y;
}

void velocity_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    Y[2] = std::sqrt(std::pow(msg->twist.linear.x, 2) + std::pow(msg->twist.linear.x, 2));
}

int main(int argc, char **argv){
    // Initialisation of the Kalman filter
    Eigen::Matrix3d Gx = 100 * Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix3d Galpha = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix3d Gbeta = 10 * Eigen::MatrixXd::Identity(3, 3);

    Eigen::Matrix3d A;
    Eigen::MatrixXd B(3, 2);
    Eigen::Matrix3d C = Eigen::MatrixXd::Identity(3, 3);
    B << 0.0, 0.0, 0.0, 0.0, 1.0, 1.0;

    // Node
    ros::init(argc, argv, "observer_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    
    // Publisher subscriber
    ros::Publisher state_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("state", 0);
    ros::Subscriber u1_subscriber = n.subscribe("u1", 1000, u1_Callback);
    ros::Subscriber u2_subscriber = n.subscribe("u2", 1000, u2_Callback);
    ros::Subscriber yaw_subscriber = n.subscribe("cap", 1000, yaw_Callback);
    ros::Subscriber cartesian_subscriber = n.subscribe("cartesian_coordinates", 1000, cartesian_Callback);
    ros::Subscriber velocity_subscriber = n.subscribe("vel", 1000, velocity_Callback);

    // Messages
    geometry_msgs::PoseWithCovarianceStamped state;
    state.header.frame_id = "map";

    // A quaternion
    tf::Quaternion q;

    // Loop rate
    ros::Rate loop_rate(10.);

    while (ros::ok()){
        // Loop rate
        ros::spinOnce();

        // Kalman Filtering
        A << 1, 0, h * cos(theta), 0, 1, h * sin(theta), 0, 0, 1 - h * abs(theta);
        kalman(X, Gx, u, Galpha, A, Y, Gbeta, C, B);

        // State Message
        state.pose.pose.position.x = X[0];
        state.pose.pose.position.y = X[1];
        q.setRPY(0, 0, theta);
        tf::quaternionTFToMsg(q, state.pose.pose.orientation);
        state_publisher.publish(state);

        loop_rate.sleep();
    }
    return 0;
}
