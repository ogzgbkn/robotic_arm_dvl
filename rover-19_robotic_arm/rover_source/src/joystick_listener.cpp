#include <iostream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>

using namespace std;


sensor_msgs::Joy joy_msg;

int button_6;

bool new_message = false;


void joystick_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    new_message = true;
    button_6 = msg->buttons[5];
    cout << button_6 << endl;
    cout << "Hello" << endl;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "talker");
	ros::NodeHandle joy_handle;

	ros::Subscriber joy_sub = joy_handle.subscribe("/joy",1,joystick_callback);
	//cout << "Hello2" << endl;
	ros::Rate loop_rate(100);

	while(ros::ok()){

		ros::spinOnce();

		//cout << axes_1 << endl;
	 
	    loop_rate.sleep();
	}

	return 0;


}