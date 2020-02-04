#include "ros/ros.h"
#include "tf/tf.h"
#include "math.h"
#include "eigen3/Eigen/Dense"
#include <vector>
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "stdlib.h"

Eigen::Vector2d u ={0.0, 0.0};
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
    ros::init(argc, argv, "simulator_node");//test
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    std::string ns = ros::this_node::getNamespace();

    // Publisher and subscriber definition
    ros::Publisher visualization_publisher = n.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
    ros::Publisher state_publisher = n.advertise<std_msgs::Float64MultiArray>("state", 0);
    //ros::Subscriber u1_subscriber = n.subscribe("commande", 1000, commande_Callback);
    ros::Subscriber u1_subscriber = n.subscribe("u1", 1000, u1_Callback);
    ros::Subscriber u2_subscriber = n.subscribe("u2", 1000, u2_Callback);
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Parameters
    std::string tf_name;
    X[0] = n_private.param<double>("x0", 0.0);
    X[1] = n_private.param<double>("y0", 0.0);
    X[2] = n_private.param<double>("theta0", 0.0);
    X[3] = n_private.param<double>("v0", 0.0);
    tf_name = n_private.param<std::string>("tf_name", "boat");

    // tf Message
    geometry_msgs::TransformStamped boat_tf;
    boat_tf.header.frame_id = "map";
    boat_tf.child_frame_id = tf_name;
    boat_tf.transform.translation.z = 0;
    // State Message

    std_msgs::Float64MultiArray pose; 


    // Quaternion
    tf::Quaternion q;

    // Visualization Message
    visualization_msgs::Marker marker;
    marker.header.frame_id = tf_name;
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    q.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
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

        
        pose.data.clear();
        std::vector<double> Xv = {X[0], X[1], X[2], X[3]};
        pose.data.insert(pose.data.end(), Xv.begin(), Xv.end());

        // tf Message
        boat_tf.header.stamp = ros::Time::now();
        boat_tf.transform.translation.x = X[0];
        boat_tf.transform.translation.y = X[1];
        q.setRPY(0.0, 0.0, X[2]);
        tf::quaternionTFToMsg(q, boat_tf.transform.rotation);
        tf_broadcaster.sendTransform(boat_tf);

        // Visualization Message
        marker.header.stamp = ros::Time();
        visualization_publisher.publish(marker);
        state_publisher.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
