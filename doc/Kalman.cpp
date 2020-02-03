#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"

using namespace std;
using namespace Eigen;

Vector2d ykal;
Vector4d ukal;

void kalman_predict(Vector4d& x1, Matrix4d& Gx1, Vector4d& xup, Matrix4d& Gup, Vector4d& u, Matrix4d& Galpha, Matrix4d& A){
    Gx1 = A * Gup * A.transpose() + Galpha;
    x1 = A * xup + u;
}

void kalman_correct(Vector4d&xup, Matrix4d& Gup, Vector4d& x0, Matrix4d& Gx0, Vector2d& y, Matrix2d& Gbeta, MatrixXd& C){
    MatrixXd S = C * Gx0 * C.transpose() + Gbeta;
    MatrixXd K = Gx0 * C.transpose() * S.inverse();
    VectorXd ytilde = y - C * x0;
    Gup = (MatrixXd::Identity(x0.size(), x0.size()) - K * C) * Gx0;
    xup = x0 + K * ytilde;
}

void kalman(Vector4d& x0, Matrix4d& Gx0, Vector4d& u, Matrix4d& Galpha, Matrix4d& A, Vector2d& y, Matrix2d& Gbeta, MatrixXd& C){
    Vector4d xup;
    Matrix4d Gup;
    kalman_correct(xup, Gup, x0, Gx0, y, Gbeta, C);
    kalman_predict(x0, Gx0, xup, Gup, u, Galpha, A);
}

void comCallback(const geometry_msgs::Vector3::ConstPtr& msg){
    ukal << 0, 0, msg->x, msg->y;
}

void GPSCallback(const geometry_msgs::Vector3::ConstPtr& msg){
    ykal << msg->x, msg->y;
}

int main(int argc, char **argv){
    //Initialisation Kalman
    // ------------------------------------------
    Vector4d x0 = {0., 0., 0., 0.1}; // doit être initialisé correctement dans le launch !
    Matrix4d Gx0 = 10 * MatrixXd::Identity(4, 4);
    Matrix4d Galpha = MatrixXd::Zero(4, 4);
    MatrixXd C(2, 4);
    C << 1., 0., 0., 0., 0., 1., 0., 0.;
    Matrix2d B;
    B << 1, -1, 1, 1;
    Matrix2d Gbeta;theta1
    Gbeta << 5, 0, 0, 5;
    // ------------------------------------------

    const double dt = 0.1;
    ros::init(argc, argv, "Kalman");
    ros::NodeHandle n;
    n.param<double>("pos_x", x0(0), 0.0);
    n.param<double>("pos_y", x0(1), 0.0);
    n.param<double>("yaw", x0(2), 0.0);
    n.param<double>("yaw", x0(3), 0.1);
    ros::Publisher estimated_state = n.advertise<geometry_msgs::PoseStamped>("xhat", 1000);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );
    ros::Subscriber sub = n.subscribe("u", 1000, comCallback); // message de type Vector3
    ros::Subscriber sub = n.subscribe("nmea", 1000, GPSCallback); // message de type Vector 3
    ros::Rate loop_rate(10.);
    tf::Quaternion q;

    while (ros::ok()){
        // Acquisition de la commande et du GPS
        ros::spinOnce();

        // MàJ de la position
        Matrix4d A;
        A << 1, 0, 0, dt * cos(x0[2]), 0, 1, 0, dt * sin(x0[2]), 0, 0, 1, 0, 0, 0, 0, 1 - dt * abs(x0[3]);
        kalman(x0, Gx0, ukal, Galpha, A, ykal, Gbeta, C);

        // Création et publication du message contenant la position estimée
        // ------------------------------------------------------------------------
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = x0(0);
        msg.pose.position.y = x0(1);
        msg.pose.position.z = x0(3); // attention : z correspond ici à la vitesse !
        q.setRPY(0, 0, x0(2));
        tf::quaternionTFToMsg(q, msg.pose.orientation);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        estimated_state.publish(msg);
        // ------------------------------------------------------------------------
        // Création du marker : à modifier pour les dimensions du robot
        // -----------------------------------------------------
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = ros::this_node::getNamespace();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = msg.pose;
        tf::quaternionTFToMsg(q, marker.pose.orientation);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.mesh_resource = "package://tp2/meshs/boat.dae";
        vis_pub.publish( marker );
        // -----------------------------------------------------
        loop_rate.sleep();
    }
    return 0;
}