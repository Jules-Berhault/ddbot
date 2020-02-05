#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"

using namespace std;

#define RATE 10

string port = "/dev/ttyUSB0";
int baudrate = 115200;
serial::Serial encoder(port, baudrate, serial::Timeout::simpleTimeout(1000));


int cast_cmd(int cmd);
void send_arduino_motor_cmd(int cmdl, int cmdr);
//void get_encoders_data(geometry_msgs::Pose2D &data);
void get_encoders_data(int &c_l, int &c_r);
void sync_encoders();

// Attention à bien inclure chaque type de message !
int main(int argc, char **argv)
{
    // Initialisation du node : le troisième argument est son nom

    cout << "Ouverture de la connection" << endl;
    if (encoder.isOpen())
    {
        cout << "Serial -> OK" << endl;
        usleep(100 * 1000);
    }
    else
    {
        cout << "** erreur encoders driver **" << endl;
        return 0;
    }

    ros::init(argc, argv, "encoders_driver_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(RATE);

    ros::Publisher odo_1 = n.advertise<std_msgs::Float64>("delta_odo_1", 10);    
    ros::Publisher odo_2 = n.advertise<std_msgs::Float64>("delta_odo_2", 10);    

    cout << "-> Lancement du driver Encoder" << endl;

    
    std_msgs::Float64 odOL;
    std_msgs::Float64 odOR;

    sync_encoders();
    int c_l, c_r, old_r, old_l;
    double t;
    double t0 = ros::Time::now().toSec();

    while (ros::ok())
    {
        
        get_encoders_data(c_l, c_r);

        cout <<(c_l - old_l)<<"  "<< abs((c_r - old_r)) << endl;
        odOL.data = (c_l - old_l);
        odOR.data = abs((c_r - old_r));
        
        old_l = c_l;
        old_r = c_r;

       odo_1.publish(odOL);
       odo_2.publish(odOR);
        

        ros::spinOnce();
        // Pause
        loop_rate.sleep();
    }

    return 0;
}

void sync_encoders()
{
    cout << "attemp to sync..." << endl;
    char b;
    bool sync_test = false;
    while (!sync_test)
    {
        b = encoder.read(1)[0];
        if (int(b) == 0xff)
        {
            cout << "v[0] : " << (int)b;
            b = encoder.read(1)[0];
            if (int(b) == 0x0d)
            {
                sync_test = true;
            }
            cout << " | v[1] : " << (int)b << endl;
        }
    }
    string data = encoder.read(15);

    cout << "Sync ok" << endl;
}

void get_encoders_data(int &c_l, int &c_r)
{
    bool sync = true;
    string data;
    string v = encoder.read(17);
    //cout << "encoders values : ";
    /*
    for(int i = 0; i < 12; i++)
    {
        cout << "v["<< i << "]=" << (int) v[i] << "."; 
    }

    cout << endl;*/

    int c1 = (int)v[0];
    int c2 = (int)v[1];

    int sensLeft;
    int sensRight;
    int posLeft;
    int posRight;
    int voltLeft;
    int voltRight;

    if (c1 != 0xff || c2 != 0x0d)
    {
        cout << "not sync..." << endl;
        sync_encoders();
    }
    else
    {
        sensLeft = (int)v[6];
        sensRight = (int)v[7];
        posRight = (int)(v[8]) * 256;
        posRight = posRight + (int)v[9];
        posLeft = (int)(v[10]) * 256;
        posLeft = posLeft + (int)v[11];
        voltLeft = (int)(v[12]) * 256;
        voltLeft = voltLeft + (int)v[13];
        voltRight = (int)(v[14]) * 256;
        voltRight = voltRight + (int)v[15];
    }

    c_l = posLeft;
    c_r = posRight;

}