#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cstdio>
#include <iostream>
#include <termios.h>
#include <signal.h>

// made with teleop_turtle_key.cpp as template and guide
// (not coppied, retyped and understood)

class TurtleWasdOp
{
public:
	TurtleWasdOp();
	// get key bindings from param server
	void getKeys();
	// input reading loop
	void keyLoop();

private:
	// get one key bind from param 'name' and load into 'c'
	void checkKeyParam(std::string name, char &c);

	ros::NodeHandle nh;
	ros::Publisher turtle_pub;
	double linear, angular, l_scale, a_scale;
	char keycode_L, keycode_R, keycode_F, keycode_B;
};

TurtleWasdOp::TurtleWasdOp():
	linear(0.0),
	angular(0.0),
	l_scale(2.0),
	a_scale(2.0),
	// default key bindings
	keycode_L('a'),
	keycode_R('d'),
	keycode_F('w'),
	keycode_B('s')
{
	nh.param("scale_angular", a_scale, a_scale);
	nh.param("scale_linear", l_scale, l_scale);

	// get key bindings from param server
	getKeys();

	turtle_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

// get one key bind from param 'name' and load into 'c'
void TurtleWasdOp::checkKeyParam(std::string name, char &c)
{
	// check for parameter 'name' on the server
	std::string tempstr = "a";
	tempstr.replace(0, 1, 1, c);
	if(nh.hasParam(name))
	{
		// load keybind, check if correct
		nh.param(name, tempstr, tempstr);
		if(tempstr.size()==1)
			c = tempstr[0];
		else
		{
			tempstr = name + " was set to more than one character\nsetting deafult\n";
			std::cerr << tempstr;
			tempstr.clear();
			tempstr.insert(0, 1, c);
			nh.setParam(name, tempstr);
		}
	}
	else
		//set param 'name' to default key
		nh.setParam(name, tempstr);
}

// get key bindings from param server
void TurtleWasdOp::getKeys()
{
	checkKeyParam("keycode_L", keycode_L);
	checkKeyParam("keycode_R", keycode_R);
	checkKeyParam("keycode_F", keycode_F);
	checkKeyParam("keycode_B", keycode_B);
}

int kfd = 0;
struct termios cooked, raw;

// input reading loop
void TurtleWasdOp::keyLoop()
{
	char c;
	bool dirty = false;

	// get raw terminal and set it to read without EOL char
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("\n+---------------------------+");
	puts("|   Reading from keyboard   |");
	puts("+---------------------------+");
	printf("|Use %c%c%c%c to move the turtle|\n", keycode_F, keycode_L, keycode_B, keycode_R);
	puts("+---------------------------+");

	// actual loop
	for(;;)
	{
		// read and check if successful
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		linear = angular = 0;
	    ROS_DEBUG("value: 0x%02X\n", c);

	    // check the keys
		if(c==keycode_L)
		{
	        ROS_DEBUG("LEFT");
			angular = 1.0;
			dirty = true;
		}
		if(c==keycode_R)
		{
	        ROS_DEBUG("RIGHT");
			angular = -1.0;
			dirty = true;
		}
		if(c==keycode_F)
		{
	        ROS_DEBUG("FORWARD");
			linear = 1.0;
			dirty = true;
		}
		if(c==keycode_B)
		{
	        ROS_DEBUG("BACKWARD");
			linear = -1.0;
			dirty = true;
		}

		// apply maths and publish on the topic
		geometry_msgs::Twist twist;
		twist.angular.z = angular*a_scale;
		twist.linear.x = linear*l_scale;
		if(dirty)
		{
			dirty = false;
			turtle_pub.publish(twist);
		}

	}
	return;
}

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "turtle_wasd_op");
	TurtleWasdOp turtle_wasd_op;

	signal(SIGINT, quit);

	turtle_wasd_op.keyLoop();

	return 0;
}
