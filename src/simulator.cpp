#include "ros/ros.h"
#include "tf/tf.h"
#include "math.h"
#include "eigen3/Eigen/Dense"

#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <geometry_msgs/TransformStamped.h>

#include "visualization_msgs/Marker.h"
#include <tf2/LinearMath/Quaternion.h>

#include "stdlib.h"

Eigen::Vector2d u = {0.0, 0.0};
Eigen::Vector4d X = {0.0, 0.0, 0.0, 0.0};

void u1_Callback(const std_msgs::Float64::ConstPtr& msg){
    u[0] = msg->data;
}

void u2_Callback(const std_msgs::Float64::ConstPtr& msg){
    u[1] = msg->data;
}

void integration_euler(Eigen::Vector4d &X, Eigen::Vector2d &u, double h) {
    Eigen::Vector4d dX = {X[3]*cos(X[2]), X[3]*sin(X[2]), u[0] - u[1], u[0] + u[1] - abs(X[3])*X[3]};
    X = X + h*dX;
}

int main(int argc, char **argv) {
    // ROS Node declaration
    ros::init(argc, argv, "simulator_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    
    // Publisher
    ros::Publisher visualization_publisher = n.advertise<visualization_msgs::Marker>("boat_marker", 1000);
    ros::Publisher state_publisher = n.advertise<geometry_msgs::PointStamped>("cartesian_coordinates", 1000);
    ros::Publisher speed_publisher = n.advertise<geometry_msgs::TwistStamped>("vel",1000);
    ros::Publisher yaw_publisher = n.advertise<std_msgs::Float64>("cap",1000);

    // Subscriber
    ros::Subscriber u1_subscriber = n.subscribe("u1", 1000, u1_Callback);
    ros::Subscriber u2_subscriber = n.subscribe("u2", 1000, u2_Callback);

    // Parameters
    std::string tf_name;
    X[0] = n_private.param<double>("x0", 0.0);
    X[1] = n_private.param<double>("y0", 0.0);
    X[2] = n_private.param<double>("theta0", 0.0);
    X[3] = n_private.param<double>("v0", 0.0);
    tf_name = n_private.param<std::string>("tf_name", "boat");

    // State Message
    geometry_msgs::PointStamped state;
    state.header.frame_id = "map";

    // Velocity Message
    geometry_msgs::TwistStamped velocity;
    velocity.header.frame_id = "map";

    // Yaw Message
    std_msgs::Float64 theta;

    // Quaternion
    tf::Quaternion q;

    // Visualization Message
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.mesh_resource = "package://ddbot/meshs/boat.dae";

    // Setting the loop rate
    double h = 1.0/25.0;
    ros::Rate loop_rate(25);

    // Loop 
    while (ros::ok()) {
        // Getting the incomming messages
        ros::spinOnce();

        // Simulating the state of the boat
        integration_euler(X, u, h);
        
        // State message
        state.header.stamp = ros::Time::now();
        state.point.x = X[0];
        state.point.y = X[1];
        state_publisher.publish(state);

        // Velocity
        velocity.header.stamp = ros::Time::now();
        velocity.twist.linear.x = X[3]*std::cos(X[2]);
        velocity.twist.linear.y = X[3]*std::sin(X[2]);
        speed_publisher.publish(velocity);

        // Yaw Message
        theta.data = X[2];
        yaw_publisher.publish(theta);

        // Visualization Message
        marker.header.stamp = ros::Time();
        marker.pose.position.x = X[0];
        marker.pose.position.y = X[1];
        q.setRPY(0.0, 0.0, X[2]);
        tf::quaternionTFToMsg(q, marker.pose.orientation);
        visualization_publisher.publish(marker);

        // Loop Rate
        loop_rate.sleep();
    }
    return 0;
}
