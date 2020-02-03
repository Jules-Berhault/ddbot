#include "ros/ros.h"
#include "tf/tf.h"
#include "eigen3/Eigen/Dense"

#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <stdlib.h>

double Lx = 10;
double Ly = 7;

Eigen::Vector2d wanted_position = {0.0, 0.0};
Eigen::Vector2d wanted_speed = {0.0, 0.0};
Eigen::Vector2d wanted_acceleration = {0.0, 0.0};

void position(Eigen::Vector2d &wanted_position, double &t) {
    wanted_position[0] = Lx*std::sin(t);
    wanted_position[1] = Ly*std::sin(2*t);
}

void speed(Eigen::Vector2d &wanted_speed, double &t) {
    wanted_position[0] = Lx*std::cos(t);
    wanted_position[1] = 2*Ly*std::cos(2*t);
}

void acceleration(Eigen::Vector2d &wanted_acceleration, double &t) {
    wanted_position[0] = -Lx*std::sin(t);
    wanted_position[1] = -4*Ly*std::sin(2*t);
}

int main(int argc, char **argv) {
    // ROS Node declaration
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    // Publisher and Subscriber
    ros::Publisher position_publisher = n.advertise<geometry_msgs::PointStamped>("wanted_position", 0);
    ros::Publisher speed_publisher = n.advertise<geometry_msgs::TwistStamped>("wanted_speed", 0);
    ros::Publisher acceleration_publisher = n.advertise<geometry_msgs::AccelStamped>("wanted_acceleration", 0);
    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Publisher visualization_publisher = n.advertise<visualization_msgs::Marker>("/target_marker", 0);

    // tf Message
    geometry_msgs::TransformStamped boat_tf;
    boat_tf.header.frame_id = "map";
    boat_tf.child_frame_id = "target";
    boat_tf.transform.translation.z = 0;

    // Visualization Message
    visualization_msgs::Marker marker;
    marker.header.frame_id = "target";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    // Wanted position message
    geometry_msgs::PointStamped w_point;
    w_point.header.frame_id = "map";
    w_point.point.z = 0.0;

    // Wanted speed message
    geometry_msgs::TwistStamped w_twist;
    w_twist.twist.linear.z = 0.0;

    // Wanted acceleration message
    geometry_msgs::AccelStamped w_accel;
    w_accel.accel.linear.z = 0.0;

    // Setting the loop rate
    ros::Rate loop_rate(25);

    // Loop 
    while (ros::ok()) {
        // Getting the incomming messages
        ros::spinOnce();

        // Wanted position message
        w_point.point.x = wanted_position[0];
        w_point.point.y = wanted_position[1];
        position_publisher.publish(w_point);

        // Wanted speed message
        w_twist.twist.linear.x = wanted_speed[0];
        w_twist.twist.linear.y = wanted_speed[1];
        speed_publisher.publish(w_twist);

        // Wanted acceleration message
        w_accel.accel.linear.x = wanted_acceleration[0];
        w_accel.accel.linear.y = wanted_acceleration[1];
        acceleration_publisher.publish(w_accel);

        // tf Message
        boat_tf.header.stamp = ros::Time::now();
        boat_tf.transform.translation.x = wanted_position[0];
        boat_tf.transform.translation.y = wanted_position[1];
        q.setRPY(0.0, 0.0, 0.0);
        tf::quaternionTFToMsg(q, boat_tf.transform.rotation);
        tf_broadcaster.sendTransform(boat_tf);

        // Visualization Message
        marker.header.stamp = ros::Time();
        visualization_publisher.publish(marker);
    }
    return 0;
}