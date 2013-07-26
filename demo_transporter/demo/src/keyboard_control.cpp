/*
 * Modified from turtlebot_key.cpp to publish cmd_pos msgs
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_Q 0x71

class KBCtrl_Transporter
{
public:
  KBCtrl_Transporter();
  void keyLoop();
private:
  ros::NodeHandle nh;
  double linear, angular;
  ros::Publisher pos_pub;
};

KBCtrl_Transporter::KBCtrl_Transporter():
  linear(0.0),angular(0.0)
{  
  pos_pub = nh.advertise<geometry_msgs::Twist>("cmd_pos", 10);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KBCtrl_Transporter");
  ros::NodeHandle n;

  KBCtrl_Transporter transporter;

  signal(SIGINT,quit);

  transporter.keyLoop();
  
  return(0);
}

void KBCtrl_Transporter::keyLoop()
{
  char c;
  bool dirty=false; //to keep track if event needs to be published

  // get the console in raw mode  
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear=angular=0.0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
		ROS_INFO("LEFT");
		angular = 90.0;
		dirty = true;
	  break;
      case KEYCODE_R:
		ROS_INFO("RIGHT");
		angular = -90.0;
		dirty = true;
	  break;
      case KEYCODE_U:
		ROS_INFO("UP");
		linear = 100.0;
		dirty = true;
	  break;
      case KEYCODE_D:
		ROS_INFO("DOWN");
		linear = -100.0;
		dirty = true;
	  break;
      case KEYCODE_Q:
		ROS_INFO("QUIT");
		linear = 0.0;
		angular = 0.0;
		dirty = true;
	  break;
      case '5':
		ROS_INFO("SPIN 360deg");
		angular = 360.0;
		dirty = true;
	  break;
      case '8':
		ROS_INFO("Move 2m");
		linear = 2000.0;
		dirty = true;
	  break;
    } //switch

	if(dirty)
	{
		printf("linear is %f and angular is %f\n",linear,angular);
		geometry_msgs::Twist pos;
		pos.linear.x=linear;
		pos.angular.z=angular;
		pos_pub.publish(pos);    
		dirty=false;
	}
    if(c==KEYCODE_Q)
      quit(SIGINT);

  } //for loop 

return;
} //KBCtrl_Transporter::keyLoop()
