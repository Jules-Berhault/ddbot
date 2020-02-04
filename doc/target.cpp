#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <math.h>
#include "std_srvs/Trigger.h"
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>
#include "tf/tf.h"
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

double u1 = 0;
double u2 = 0;
double dt = 0.04; 

using namespace std; 


void integration_euler(vector<double> &X, double &u1, double &u2,  double &dt)
{
    
    
    
    X[0] = X[0] + u1*dt;
    X[1] = X[1] + u2*dt;

     
}


void commandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u1  = 10*msg->linear.x;
    u2  = 10*msg->angular.z;
}


int main(int argc, char **argv)
{
    
    
    ros::init(argc, argv, "Target");
            
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("targetPos", 10);
    geometry_msgs::PoseStamped targetState;
    ros::Subscriber sus = n.subscribe("command",1000, &commandCallback);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    visualization_msgs::Marker marker;
    geometry_msgs::Twist command_u;
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::TransformBroadcaster br;

    double posx, posy;
    n.param<double>("targetx", posx, -5);
    n.param<double>("targety", posy, 5);

    vector<double> X = {posx, posy};
    
    ros::Rate loop_rate(25);
    targetState.pose.position.x = posx;
    targetState.pose.position.y = posy;
    tf::Quaternion q;
    q.setRPY(M_PI*0.5,0 , 0);
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "cible";


    tf::quaternionTFToMsg(q, targetState.pose.orientation);
    while (ros::ok()){
    
        targetState.header.stamp = ros::Time::now();
        transformStamped.header.stamp = ros::Time::now();


        targetState.header.frame_id = "map";

        integration_euler(X, u1, u2, dt);
        targetState.pose.position.x = X[0];
        targetState.pose.position.y = X[1];

        transformStamped.transform.translation.x = X[0];
        transformStamped.transform.translation.y = X[1];

        transformStamped.transform.rotation.x = 0.;
        transformStamped.transform.rotation.y = 0.;
        transformStamped.transform.rotation.z = 0.;
        transformStamped.transform.rotation.w = 1.;

        
        marker.header.frame_id = "cible";
        marker.header.stamp = ros::Time();
        marker.ns = "cible";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        
	    
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        
        
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0; // alpha = transparence
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.mesh_resource = "package://tp2/meshes/lighthouse.obj";
        vis_pub.publish( marker);
        pub.publish(targetState);
        br.sendTransform(transformStamped);
        
        ros::spinOnce();
        loop_rate.sleep();

    }


return 0;
}