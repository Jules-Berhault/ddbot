#include "ros/ros.h"
#include "tf/tf.h"
#include "math.h"
#include "eigen3/Eigen/Dense"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64MultiArray.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "stdlib.h"
#include "iostream"

int main(int argc, char **argv) {
    ros::init(argc, argv, "simulator_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    ros::Publisher state = n.advertise<geometry_msgs::PoseStamped>("boat_state", 1000);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 0 );
    ros::Subscriber command = n.subscribe("boat_command", 1000, commandCallback);

    std::string ns = ros::this_node::getNamespace();

    tf2_ros::TransformBroadcaster tf_broadcast;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = ns;
    
    double x0, y0, theta0;
    std::string tf_name;
    x0 = n_private.param<double>("x0", 20.0);
    y0 = n_private.param<double>("y0", 20.0);
    theta0 = n_private.param<double>("theta0", 0.0);
    tf_name = n_private.param<std::string>("tf_name", ns);

    // Creating the message
    geometry_msgs::PoseStamped msg_state;

    // Quaternion
    tf::Quaternion q;
    
    q.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(q, msg_state.pose.orientation);
    msg_state.header.stamp = ros::Time::now();
    msg_state.header.frame_id = tf_name;
    msg_state.pose.position.z = 0.0;

    Eigen::Vector3d X = {x0, y0, theta0};
    double h = 1.0/25.0;

    ros::Rate loop_rate(25);

    while (ros::ok()) {
        // Getting the incomming messages
        ros::spinOnce();

        // Simulating the state of the boat
        integration_euler(X, u, h);

        // Creating the tf for the boat
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.child_frame_id = tf_name;
        transformStamped.transform.translation.x = X[0];
        transformStamped.transform.translation.y = X[1];
        transformStamped.transform.translation.z = 0;
        q.setRPY(0.0, 0.0, X[2]);
        tf::quaternionTFToMsg(q, transformStamped.transform.rotation);

        tf_broadcast.sendTransform(transformStamped);

        // State Message
        msg_state.pose.position.x = X[0];
        msg_state.pose.position.y = X[1];
        q.setRPY(0.0, 0.0, X[2]);
        tf::quaternionTFToMsg(q, msg_state.pose.orientation);

        // Publishing the message
        state.publish(msg_state);

        visualization_msgs::Marker marker;
        marker.header.frame_id = tf_name;
        marker.header.stamp = ros::Time();
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
        // marker.color.r = 1.0;
        // marker.color.g = 1.0;
        // marker.color.b = 1.0;
        marker.mesh_resource = "package://tp2/meshs/auv.dae";
        vis_pub.publish(marker);

        loop_rate.sleep();
    }
    return 0;
}
