#include "ros/ros.h"
#include "tf/tf.h"

#include "eigen3/Eigen/Dense"
#include <stdlib.h>
#include <math.h>

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <geometry_msgs/AccelStamped.h>

#include "geometry_msgs/Twist.h" // Maybe to transform into a TwistStamped
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <visualization_msgs/Marker.h>

// State Vector and position
Eigen::Vector4d X = {0.0, 0.0, 0.0, 0.1};
Eigen::Vector2d u;
Eigen::Vector2d w = {5, 5}; 
Eigen::Vector2d dw = {0, 0}; 
Eigen::Vector2d ddw = {0, 0};

// Coefficients
double kp, kd; 

void stateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    X[0] = msg->pose.position.x;
    X[1] = msg->pose.position.y;
} 

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    double vx = msg->twist.linear.x;
    double vy = msg->twist.linear.y;
    X[3] = std::sqrt(vx*vx +vy*vy);
}

void yawCallback(const std_msgs::Float64::ConstPtr& msg) {
    X[2] = msg -> data*M_PI/180;
}

void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    w[0] = msg->point.x;
    w[1] = msg->point.y;
}

void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    dw[0] = msg->twist.linear.x;
    dw[1] = msg->twist.linear.y;
}

void accelerationCallback(const geometry_msgs::AccelStamped::ConstPtr& msg) {
    ddw[0] = msg->accel.linear.x;
    ddw[1] = msg->accel.linear.y;
}

void gainCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    kp = msg->linear.x;
    kd = msg->angular.z;
    ROS_WARN("kp %f", kp);
    ROS_WARN("kd %f", kd);
}

void command(Eigen::Vector2d &u) {
    // Return the tresholded command between 0 and 255 for sending pwm on motors
    double M = std::max(u[0], u[1]);
    double r1 = u[0]/M;
    double r2 = u[1]/M;
    double k = std::min(1.0, M/255.0);

    u[0] = 255*std::max(2 / (1 + std::exp(-3*r1*k)) - 1, 0.0);
    u[1] = 255*std::max(2 / (1 + std::exp(-3*r2*k)) - 1, 0.0);
}

void control(Eigen::Vector2d &w, Eigen::Vector2d &dw, Eigen::Vector2d &u, std_msgs::String &fsm) {   
    double x = X[0]; 
    double y = X[1];
    double theta = X[2];
    double v = X[3];

    double st = std::sin(theta);
    double ct = std::cos(theta);

    Eigen::Matrix2d A;
    A << ct - v*st, ct + v*st,
         st + v*ct, st - v*ct; 

    Eigen::Vector2d B = {-std::abs(v)*v*ct, -std::abs(v)*v*st};
    Eigen::Vector2d Y = {x, y};
    Eigen::Vector2d dY = {v*ct, v*st};
    Eigen::Vector2d a = {1, 2};
    Eigen::Vector2d b = {2, 3};
    Eigen::Vector2d z;

    // z = 2*(w - Y) + 2*(dw - dY);
    z = kp*(w - Y) + kd*(dw - dY)+ddw;

    // Preventing the singularity of A
    if (std::abs(A.determinant()) < 0.0001) {
        u = {20.0, 10.0};
        fsm.data = "Singular Matrix";
    }
    else {
        u = A.fullPivLu().solve(z - B);
        // If the boat want to go back, then turning left
        if ((w[0] - X[0]) * std::cos(X[2]) + (w[1] - X[1]) * std::sin(X[2]) < 0) {
            u = {20.0, 10.0};
        	fsm.data = "Turning";
        }
        else {
            command(u);  
            fsm.data = "Regulation";      
        }
    }


    ROS_WARN("Setting commands at {%f; %f}", u[0], u[1]);
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "pilot");

    ros::NodeHandle n;
    
    //Suscriber
    ros::Subscriber wanted_suscribe = n.subscribe("wanted_position", 1000, &positionCallback);
    ros::Subscriber wanted_speed_suscribe = n.subscribe("wanted_speed", 1000, &speedCallback);
    ros::Subscriber wanted_acceleration_suscribe = n.subscribe("wanted_acceleration", 1000, &accelerationCallback);
    ros::Subscriber gains_suscribe = n.subscribe("/cmd_vel", 1000, &gainCallback);
    ros::Subscriber state_subscribe = n.subscribe("state", 1000, &stateCallback);
    //ros::Subscriber rstate_subscribe = n.subscribe("cartesian_coordinates", 1000, &rstateCallback);
    ros::Subscriber velocity_subscribe = n.subscribe("vel", 1000, &velocityCallback);
    ros::Subscriber yaw_subscribe = n.subscribe("cap", 1000, &yawCallback);
    
    //Publisher
    ros::Publisher u1_pub = n.advertise<std_msgs::Float64>("u1", 10);
    ros::Publisher u2_pub = n.advertise<std_msgs::Float64>("u2", 10);

    // Publisher for the state of the ddbot
    ros::Publisher fsm_publish = n.advertise<std_msgs::String>("fsm", 1000);
    
    std_msgs::Float64 u1;
    std_msgs::Float64 u2;

    n.param<double>("kp", kd, 0);
    n.param<double>("kd", kp, 0);

    std_msgs::String fsm;
    
    ros::Rate loop_rate(25);
    
    while (ros::ok()) {
        control(w, dw, u, fsm);
        fsm_publish.publish(fsm);
        u1.data = u[0];
        u2.data = u[1];
        u1_pub.publish(u1);
        u2_pub.publish(u2);
        ros::spinOnce();
        loop_rate.sleep();
    }
return 0;
}
