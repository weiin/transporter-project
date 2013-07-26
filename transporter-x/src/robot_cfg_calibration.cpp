/*
 * Measurements of the robot wheel base and wheel diameter are not accurate
 * This program is for manually calibrating the correction factors
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

class calibrate_Transporter
{
public:
  calibrate_Transporter();
  void mainMenu();
  void calibrateWheelCircum();
  void calibrateWheelBase();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  double linear, angular;
  ros::Publisher vel_pub;
  ros::Publisher pos_pub;
  ros::Subscriber odom_sub;
  bool started, recalc;
  ros::Time start_time;
  int cnt;
  double time_taken, k,b;
  geometry_msgs::Twist vel, pos;
  double cur_x, cur_y, cur_theta;
};

calibrate_Transporter::calibrate_Transporter():
  linear(0.0),angular(0.0),started(false),recalc(false)
{  
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pos_pub = nh.advertise<geometry_msgs::Twist>("cmd_pos", 1);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &calibrate_Transporter::odomCallback,this);

  ros::param::get("/transporter/wheel_circum_correction", k);
  ros::param::get("/transporter/wheel_base_correction", b);
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
  ros::init(argc, argv, "calibrate_Transporter");
  calibrate_Transporter transporter;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&calibrate_Transporter::mainMenu, &transporter));
  
  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;  
  return(0);
}

void calibrate_Transporter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_DEBUG("Currently at x [%f] & y [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
  cur_x = msg->pose.pose.position.x;
  cur_y = msg->pose.pose.position.y;
  cur_theta = 2.0 * acos(msg->pose.pose.orientation.w);
}

void calibrate_Transporter::mainMenu()
{
  char c;

  // get the console in raw mode  
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  while (ros::ok())
  {
	puts("---------");
	puts("Main Menu");
	puts("---------");
	puts("'c' to calibrate wheel circumference");
	puts("'b' to calibrate wheel base");
	puts("'q' to quit");

    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

	switch (c)
	{
		case 'c':
		  calibrate_Transporter::calibrateWheelCircum();
		  break;
		case 'b':
		  calibrate_Transporter::calibrateWheelBase();
		  break;
		case 'q':
		  quit(SIGINT);
	} // switch
  }
return;
} //calibrate_Transporter::mainMenu()

void calibrate_Transporter::calibrateWheelCircum()
{
  char c;
  cnt = 0;

  while (ros::ok())
  {
	puts("**** Calibrating for wheel circumference error ****");
	puts("**** Robot will alternate forward and reverse motion ****");
	puts("Press 's' to start straight line motion");
	puts("Press 's' again to stop the robot at 1m");
	puts("Press 'q' to stop calibration and return to main menu");

    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

	if(started && c=='s')
	{
		started=false;
		linear=0.0;
		recalc=true;
		cnt++;
	}
	else if(!started && c=='s')
	{
		started=true;
		linear=0.1*((double)pow(-1,cnt));
		start_time=ros::Time::now();
	}
	else if(c=='q')
	{
		started=false;
		linear=0.0;
		angular=0.0;
	}

	vel.linear.x=linear;
	vel.angular.z=angular;
	vel_pub.publish(vel);

	if(c=='q') return;

	if(recalc)
	{
		recalc=false;
		time_taken= (ros::Time::now()-start_time).toSec();
		printf("time taken= %f [theoretical time taken should be 10s]\n",time_taken);
		printf("use wheel_circum_correction of [ %f ]\n",k*10.0/time_taken);
	}
  } // while (ros::ok())
} // calibrate_Transporter::calibrateWheelCircum()

void calibrate_Transporter::calibrateWheelBase()
{
  char c;
  cnt = 0;

  while (ros::ok())
  {
	puts("**** Calibrating for wheel base error ****");
	puts("**** Robot will alternate clockwise and anticlockwise motion ****");
	puts("Press 's' to start rotation");
	puts("Press 's' again to stop the robot after 360deg");
	puts("Press 'q' to stop calibration and return to main menu");

    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

	if(started && c=='s')
	{
		started=false;
		angular=0.0;
		recalc=true;
		cnt++;
	}
	else if(!started && c=='s')
	{
		started=true;
		angular=M_PI/10.0*((double)pow(-1,cnt));
		start_time=ros::Time::now();
	}
	else if(c=='q')
	{
		started=false;
		linear=0.0;
		angular=0.0;
	}

	vel.linear.x=linear;
	vel.angular.z=angular;
	vel_pub.publish(vel);

	if(c=='q') return;

	if(recalc)
	{
		recalc=false;
		time_taken= (ros::Time::now()-start_time).toSec();
		printf("time taken= %f [theoretical time taken should be 20s]\n",time_taken);
		printf("use wheel_base_correction of [ %f ]\n",b*time_taken/20.0);
	}
  } // while (ros::ok())
} // calibrate_Transporter::calibrateWheelBase()
