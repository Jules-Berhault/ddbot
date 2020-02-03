#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "tf/tf.h"
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include "Message_consigne.h" 
/*
  Warning : penser à inclure ce type de message dans le CMakeListe
*/
using namespace std;
using namespace Eigen;

double x_boat, y_boat, heading_boat, vitesse_boat;
Vector2d w, dw, ddw;
Vector4d vecteur_etat;
const double T = 1; // constante de convergence


Vector2d command(Vector4d& x, Vector2d& w, Vector2d& dw, Vector2d& ddw, const double T){
    Matrix2d A;
    A << cos(x[2]) -x[3] * sin(x[2]), cos(x[2]) + x[3] * sin(x[2]),
    sin(x[2]) + x[3] * cos(x[2]), sin(x[2]) - x[3] * cos(x[2]);
    Vector2d b = {-x[3] * abs(x[3]) * cos(x[2]), -x[3] * abs(x[3]) * sin(x[2])};
    Vector2d y = {x[0], x[1]};
    Vector2d dy = {x[3] * cos(x[2]), x[3] * sin(x[2])};
    Vector2d v = pow(1.0/T, 2) * (w - y) + (2.0/T) * (dw - dy) + ddw;
    Vector2d u = A.inverse() * (v - b);
    return u;
}

void consignCallback(const Message_consigne& msg){
    w << msg->w[0], msg->w[1];
    dw << msg->dw[0], msg->dw[1];
    ddw << msg->ddw[0], msg->ddw[1];
}

void stateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    /*
      Permet de récupérer le vecteur d'état après Kalman
    */
    vecteur_etat <<  msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation), msg->pose.position.z;
    //on met dans z la vitesse estimée du bateau
}

int main(int argc, char **argv){

    ros::init(argc, argv, "control_bateau");
    ros::NodeHandle n;
    ros::Publisher control = n.advertise<geometry_msgs::Vector3>("u", 1000);
    ros::Subscriber sub = n.subscribe("command", 1000, consignCallback);
    ros::Subscriber sub2 = n.subscribe("xhat", 1000, stateCallback);
    ros::Rate loop_rate(25.);

    while (ros::ok()){
        ros::spinOnce();
        /*
          Partie commande
        */
        Vector2d u;
        double u1, u2;
        
        u = command(vecteur_etat, w, dw, ddw, T);
        u1 = u[0];
        u2 = u[1];
        /*
          Publication des commandes moteurs
        */
        geometry_msgs::Vector3 msg_control;
        msg_control.x = u1; //commande moteur 1
        msg_control.y = u2; //commande moteur 2
        control.publish(msg_control);

        loop_rate.sleep();
    }
    return 0;
}