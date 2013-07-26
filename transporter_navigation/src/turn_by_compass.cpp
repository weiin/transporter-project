#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include "techx_comms/RobotState.h"

double yaw=0;
bool first_yaw_rcvd=false;

void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
	if(!first_yaw_rcvd) first_yaw_rcvd = true;
	yaw = tf::getYaw(imu_msg.orientation);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turn_by_compass");
	ros::NodeHandle nh;
	ros::Rate r(5);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Publisher state_pub = nh.advertise<techx_comms::RobotState>("state", 1, true);
	ros::Subscriber imu_subscriber = nh.subscribe("imu/data", 10, imuMsgCallback);
	ros::ServiceClient reset_odom_client = nh.serviceClient<std_srvs::Empty>("reset_odom");
	techx_comms::RobotState state;

	std_srvs::Empty srv;
	geometry_msgs::Twist vel;
	double dir, Kp;
	bool do_init_turn;
	nh.getParam("turn_by_compass/direction", dir);
	nh.getParam("turn_by_compass/Kp", Kp);
	nh.getParam("turn_by_compass/do_init_turn", do_init_turn);

	while(do_init_turn && ros::ok())
	{
		ros::spinOnce();
		if(!first_yaw_rcvd) continue;
		vel.angular.z = (dir - yaw) * Kp;
		vel.angular.z = std::min( vel.angular.z, 0.5);
		vel.angular.z = std::max( vel.angular.z, -0.5);

		ROS_INFO ("yaw = %f, dir = %f, Kp = %f, vel = %f", yaw, dir, Kp, vel.angular.z);
		vel_pub.publish(vel);    
		if(fabs(vel.angular.z)<0.005) break;
		r.sleep();
	}

	vel.angular.z = 0;
	vel_pub.publish(vel);

	reset_odom_client.waitForExistence();
	reset_odom_client.call(srv);
	
	while (!nh.hasParam("hokuyo/start_pub"))
	{
		ROS_WARN("Waiting for hokuyo laser to be started...");
		sleep(1); // check every second
	}
	nh.setParam("publish_laser",true);
	
	state.header.stamp = ros::Time::now();
	state.header.frame_id = "turn_init";
	state.state = techx_comms::RobotState::SENDNEXT_WAYPOINT;
	state_pub.publish(state);
	usleep(500000);
	
	ROS_INFO ("finished at yaw = %f",yaw);
	return 0;
}

