#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "CLIPSCore.h"

/* ** ***************************************************************
*
* Globals and prototypes
*
*  ** **************************************************************/
const std::string kbfile =
	// "/src/expert_system/oracle.dat";
	// "/src/action_planner/ViRBot_Cubes_ROS/ROS_cubes.dat";
	// "/src/action_planner/ViRBot_Cubes_ROS/ROS_virbot.dat";
	"cubes.clp";

ros::Subscriber sub;
ros::Publisher  pub;

void setup_CLIPS();
void setup_ROS(int argc, char **argv);
bool file_exists(std::string filepath);

/* ** ***************************************************************
*
* Main entry point
*
*  ** **************************************************************/
int main(int argc, char **argv){

	setup_CLIPS();
	setup_ROS(argc, argv);
	// Set spin speed to ~25Hz
	ros::Rate loop_rate(25);

	for (char i = 0; i < 25 && ros::ok(); ++i){
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Start clips
	CLIPSCore::runCLIPS(true);


	while (ros::ok()){
		// Read CLIPS output (if any)
		std::string s = CLIPSCore::getInstance().readOut();
		// Print and publish if out is not empty
		if( ! s.empty() ) {
			std_msgs::String msg;
			msg.data = std::string(s);
			pub.publish(msg);
			ROS_INFO("CLIPS says: %s", s.c_str());
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}



/* ** ***************************************************************
*
* Function definitions
*
*  ** **************************************************************/

void clips_inCallback(const std_msgs::String::ConstPtr& msg){
	// Whenever a string arrives, it is injected into CLIPS
	ROS_INFO("Received: %s", msg->data.c_str());
	CLIPSCore::strQueryKDB(msg->data.c_str());
}

void setup_CLIPS(){
	// std::string result;
	std::cout << "Starting CLIPS" << std::endl;
	ROS_INFO("Starting CLIPS");

	//This functions loads initial facts and rules from a file
	// The first parameter is a file that contains the names of CLIPS files *.clp
	// The second parameter indicates with false do not start executing the loaded files
	// The third parameter is a timeout

	if(!kbfile.empty() && file_exists(kbfile)){
		ROS_INFO("CLIPS file: %s", kbfile.c_str());
		bool init_kdb = CLIPSCore::initKDB(kbfile, false, 2000);
		if(!init_kdb){
			ROS_INFO("CLIPS error: file '%s' not found", kbfile.c_str());
			std::exit(-1);
		}
	}

	// Automatically reset CLIPs
	CLIPSCore::resetCLIPS(true);
}

void setup_ROS(int argc, char **argv){
	// Init ros
	ros::init(argc, argv, "clips_node");
	// Handler for the node
	ros::NodeHandle n;
	// Subscription to the clips_in topic. Q-size: 10
	sub = n.subscribe("clips_in", 10, clips_inCallback);
	// Publisher to write on the clips_out topic. Q-size: 100
	pub = n.advertise<std_msgs::String>("clips_out", 100);
}

bool file_exists(std::string filepath){
	struct stat buffer;
	return ( stat(filepath.c_str(), &buffer) == 0 );
}