#include <cstdlib>
#include <string>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"

/* ** ***************************************************************
*
* Globals and prototypes
*
*  ** **************************************************************/

ros::Subscriber sub;
ros::Publisher  pub;

void console_in();
void setup_ROS(int argc, char **argv);

/* ** ***************************************************************
*
* Main entry point
*
*  ** **************************************************************/
int main(int argc, char **argv){
	setup_ROS(argc, argv);
	// Set spin speed to ~10Hz
	ros::Rate loop_rate(10);

	std::thread th(console_in);

	ros::spin();

	th.join();
	return 0;
}



/* ** ***************************************************************
*
* Function definitions
*
*  ** **************************************************************/

void clips_outCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("Received: [%s]", msg->data.c_str());
}

void console_in(){
	for (std::string line; std::getline(std::cin, line);) {
		std_msgs::String msg;
		msg.data = line;
		pub.publish(msg);
		ROS_INFO("Sent: [%s]", line.c_str());
	}
}

void setup_ROS(int argc, char **argv){
	// Init ros
	ros::init(argc, argv, "clips_console");
	// Handler for the node
	ros::NodeHandle n;
	// Subscription to the clips_out topic. Q-size: 10
	sub = n.subscribe("clips_out", 10, clips_outCallback);
	// Publisher to write on the clips_in topic. Q-size: 10
	pub = n.advertise<std_msgs::String>("clips_in", 10);
}
