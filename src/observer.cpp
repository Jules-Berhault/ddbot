#include "ros/ros.h"
#include "tf/tf.h"
#include <eigen3/Eigen/Dense>
#include "stdlib.h"
#include "cmath"

#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

void kalman_predict(Eigen::Vector2d& x1, Eigen::Matrix2d& Gx1, Eigen::Vector2d& xup, Eigen::Matrix2d& Gup, Eigen::Vector2d& u, Eigen::Matrix2d& Galpha, Eigen::Matrix2d& A,  Eigen::Matrix2d& B) {
    Gx1 = A * Gup * A.transpose() + Galpha;
    x1 = A * xup + u;
}

void kalman_correct(Eigen::Vector2d&xup, Eigen::Matrix2d& Gup, Eigen::Vector2d& x0, Eigen::Matrix2d& Gx0, Eigen::Vector2d& y, Eigen::Matrix2d& Gbeta, Eigen::Matrix2d& C) {
    Eigen::MatrixXd S = C * Gx0 * C.transpose() + Gbeta;
    Eigen::MatrixXd K = Gx0 * C.transpose() * S.inverse();
    Eigen::VectorXd ytilde = y - C * x0;
    Gup = (Eigen::MatrixXd::Identity(x0.size(), x0.size()) - K * C) * Gx0;
    xup = x0 + K * ytilde;
}

void kalman(Eigen::Vector2d& x0, Eigen::Matrix2d& Gx0, Eigen::Vector2d& u, Eigen::Matrix2d& Galpha, Eigen::Matrix2d& A, Eigen::Vector2d& y, Eigen::Matrix2d& Gbeta, Eigen::Matrix2d& C, Eigen::Matrix2d& B) {
    Eigen::Vector2d xup;
    Eigen::Matrix2d Gup;
    kalman_correct(xup, Gup, x0, Gx0, y, Gbeta, C);
    kalman_predict(x0, Gx0, xup, Gup, u, Galpha, A, B);
}

// Vectors of the system
Eigen::Vector2d X = {0.0, 0.0};
Eigen::Vector2d Y = {0.0, 0.0};
Eigen::Vector2d u = {0.0, 0.0};
double theta = 0.0;
double v = 0.0;
double h = 1/10;


void yaw_Callback(const std_msgs::Float64::ConstPtr& msg){
    theta = msg->data;
}

void cartesian_Callback(const geometry_msgs::PointStamped::ConstPtr& msg){
    Y[0] = msg->point.x;
    Y[1] = msg->point.y;
}
 void velocity_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    v = std::sqrt(std::pow(msg->twist.linear.x, 2) + std::pow(msg->twist.linear.x, 2));
} 

int main(int argc, char **argv){

    
    

    // Node
    ros::init(argc, argv, "observer_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    // Loop rate
    ros::Rate loop_rate(10.);

    // Initialisation of the Kalman filter
    Eigen::Matrix2d Gx = 100 * Eigen::MatrixXd::Identity(2, 2);
    Eigen::Matrix2d Galpha = Eigen::MatrixXd::Zero(2, 2);
    Eigen::Matrix2d Gbeta = 100 * Eigen::MatrixXd::Identity(2, 2);

    Eigen::Matrix2d A = (1+h)*Eigen::MatrixXd::Identity(2, 2);
    Eigen::Matrix2d B = Eigen::MatrixXd::Identity(2, 2);
    Eigen::Matrix2d C = Eigen::MatrixXd::Identity(2, 2);

    
    // Publisher subscriber
    ros::Publisher state_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("state", 0);
    
    ros::Subscriber yaw_subscriber = n.subscribe("cap", 1000, yaw_Callback);
    ros::Subscriber cartesian_subscriber = n.subscribe("cartesian_coordinates", 1000, cartesian_Callback);
    ros::Subscriber velocity_subscriber = n.subscribe("vel", 1000, velocity_Callback);

    // Messages
    geometry_msgs::PoseWithCovarianceStamped state;
    state.header.frame_id = "map";

    // A quaternion
    tf::Quaternion q;

    

    while (ros::ok()){
        // Loop rate
        
        ros::spinOnce();
        u = {h*v*std::cos(theta), h*v*std::sin(theta)};
        // Kalman Filtering
        kalman(X, Gx, u, Galpha, A, Y, Gbeta, C, B);

        // State Message
        state.pose.pose.position.x = X[0];
        state.pose.pose.position.y = X[1];
        ROS_WARN("erreur %f ", X[0]);
        q.setRPY(0, 0, theta);
        tf::quaternionTFToMsg(q, state.pose.pose.orientation);
        state_publisher.publish(state);

        loop_rate.sleep();
    }
    return 0;
}
