#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <unistd.h>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <math.h>
#include "std_srvs/Trigger.h"
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>
#include "tf/tf.h"
#include "eigen3/Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"
using namespace std; 

double theta, posx, posy; 
double kp, kd; 


Eigen::Vector4d X = {0.0, 0.0, 0.0, 0.0};
Eigen::Vector2d w = {5, 5}; 
    Eigen::Vector2d dw = {0, 0}; 

void infosCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    X[0] = msg->data[0];
    X[1] = msg->data[1];
    X[2] = msg->data[2];
    X[3] = msg->data[3];

}

void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    w[0] = msg->point.x;
    w[1] = msg->point.y;

}

void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    dw[0] = msg->twist.linear.x;
    dw[1] = msg->twist.linear.y;

}

void testCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double u, v;
    u = msg->linear.x;
    v = msg->angular.z;
    kp = u; 
    kd = v; 
    ROS_WARN("kp %f", kp);
    ROS_WARN("kd %f", kd);
    


    //ROS_WARN("message x : %f", u);

}



void control(Eigen::Vector2d &w, Eigen::Vector2d &dw, Eigen::Vector2d &u)
{   
    double x = X[0]; 
    double y = X[1];

    //ROS_WARN("x : %f", x);
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
    z = kp*(w - Y) + kd*(dw - dY);
    u = A.fullPivLu().solve(z - B);
}



int main(int argc, char **argv)
{   
    ros::init(argc, argv, "pilot");

    ros::NodeHandle n;
    ros::Subscriber state_suscribe = n.subscribe("state", 1000, &infosCallback);
    ros::Subscriber wanted_suscribe = n.subscribe("wanted_position", 1000, &positionCallback);
    ros::Subscriber wanted_speed_suscribe = n.subscribe("wanted_speed", 1000, &speedCallback);
    ros::Subscriber test = n.subscribe("/cmd_vel", 1000, &testCallback);
    
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("commande", 10);
    
    geometry_msgs::Twist command_u;
    
    ros::Rate loop_rate(25);
    
    Eigen::Vector2d u; 
    

    while (ros::ok())
    {
        
        control(w, dw, u);

        
        command_u.linear.x = u[0];
        command_u.linear.y = u[1];

        pub.publish(command_u);
        ros::spinOnce();
        loop_rate.sleep();
        
    }


return 0;
}