#include "ros/ros.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <string>
#include <iostream>
#include <iomanip>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"

int main(int argc, char **argv) {
	FILE *lsofFile_p = popen("~/cpp/Driver_IMU/minimu9-ahrs --output euler", "r");
	if (!lsofFile_p) {
		return -1;
	}

	// Node declaration
	ros::init(argc, argv, "Node_cap");
	ros::NodeHandle n;

	// Publisher
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("cap", 1000);

	// Rate Loop
	ros::Rate loop_rate(2500);

	while (ros::ok()) {
		std_msgs::Float64 msg;
		char buffer[100];
		char *line_p = fgets(buffer, sizeof(buffer), lsofFile_p);
		float data_cap = strtof((buffer),0);
		msg.data = data_cap*M_PI/180;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	pclose(lsofFile_p);
	return 0;
}