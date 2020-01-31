#include "ros/ros.h"
#include <WGS84toCartesian.hpp>
#include "eigen3/Eigen/Dense"

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nmea_msgs/Gppga.hpp>
#include <geometry_msgs/Point.h>

Eigen::Vector2d nmea = {0.0, 0.0};

void nmea_Callback(const std_msgs::Gppga::ConstPtr& msg){
    nmea[0] = msg->lat;
    nmea[1] = msg->long;
}

void 
    // Projection with the Mercator projection
    array<double, 2> result{wgs84::toCartesian({48.4029251,-4.4690661}, {latitude, longitude})};

int main(int argc, char **argv) {

    // Setting the loop rate
    double h = 1.0/25.0;
    ros::Rate loop_rate(25);

    // Publisher and Subscriber
    ros::Publisher position_publisher = n.advertise<geometry_msgs::Point>("cartesian_coordinates", 0);
    ros::Subscriber nmea_subscriber = n.subscribe("nmea_coordinates", 1000, nmea_Callback);


    // Loop 
    while (ros::ok()) {

    }
    return 0;
}