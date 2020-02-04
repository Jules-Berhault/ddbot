#include "ros/ros.h"
#include "WGS84toCartesian.hpp"
#include "eigen3/Eigen/Dense"

#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"

#include <stdlib.h>

Eigen::Vector2d nmea = {0.0, 0.0};
Eigen::Vector2d nmea_reference = {0.0, 0.0};
Eigen::Vector2d cartesian = {0.0, 0.0};

void nmea_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    nmea[0] = msg->latitude;
    nmea[1] = msg->longitude;
}


int main(int argc, char **argv) {
    // ROS Node declaration
    ros::init(argc, argv, "projection_node");//test
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    // Publisher and Subscriber
    ros::Publisher position_publisher = n.advertise<geometry_msgs::PointStamped>("cartesian_coordinates", 0);
    ros::Subscriber nmea_subscriber = n.subscribe("nmea_coordinates", 1000, nmea_Callback);

    // Reference point for nmea projection in parameter
    nmea_reference[0] = n_private.param<double>("reference_latitude", 48.199334);
    nmea_reference[1] = n_private.param<double>("reference_longitude", -3.015625);

    // Coordinate position message
    geometry_msgs::PointStamped cartesian_point;
    cartesian_point.header.frame_id = "boat";
    cartesian_point.point.z = 0.0;

    // Setting the loop rate
    ros::Rate loop_rate(25);

    // Loop 
    while (ros::ok()) {
        // Getting the incomming messages
        ros::spinOnce();

        // Projection with the Mercator projection
        std::array<double, 2> result{wgs84::toCartesian({nmea_reference[0], nmea_reference[1]}, {nmea[0], nmea[1]})};
        cartesian[0] = result[0];
        cartesian[1] = result[1];

        // Publishing the cartesian coordinates
        cartesian_point.header.stamp = ros::Time::now();
        cartesian_point.point.x = cartesian[0];
        cartesian_point.point.y = cartesian[1];

        position_publisher.publish(cartesian_point);

    }
    return 0;
}