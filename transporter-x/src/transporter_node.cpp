/*
 * Node that listens to cmd_vel and cmd_pos msgs
 * and publishes the robot odometry
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include "transporter/transporter_driver.h"

class TransporterNode
{
public:
  TransporterNode();
  Transporter trobot;
  double cur_x,cur_y,cur_theta;
  double linear_, angular_, joylinear_, joyangular_;
  bool errorState, estopped, sensor, resetting;
  void sensorCallback(const std_msgs::Bool::ConstPtr& msg);
  void estopCallback(const std_msgs::Bool::ConstPtr& msg);
  void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void posCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void joyCallback(const geometry_msgs::Twist::ConstPtr& msg);
  bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
  void publish_odom();
  void restartRobot();
private:
  ros::NodeHandle nh,ph;
  double cov_x, cov_y, cov_th;
  double cov_vx, cov_vy, cov_vth;
  geometry_msgs::Pose2D prevPose;
  ros::Subscriber sensor_sub, estop_sub, vel_sub, pos_sub, joyvel_sub;
  ros::Publisher odom_pub, js_pub;
  ros::ServiceServer reset_odom_srv;
  ros::Time current_time, last_time;
};

TransporterNode::TransporterNode():
  cur_x(0.0),cur_y(0.0),cur_theta(0.0),linear_(0.0),angular_(0.0), joylinear_(0.0),joyangular_(0.0),
  errorState(false), estopped(false), sensor(false), resetting(false), ph("~")
{
  sensor_sub = nh.subscribe<std_msgs::Bool>("step_sensor/step", 1, &TransporterNode::sensorCallback,this);
  estop_sub = nh.subscribe<std_msgs::Bool>("cmd_estop", 1, &TransporterNode::estopCallback,this);
  vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &TransporterNode::velCallback,this);
  pos_sub = nh.subscribe<geometry_msgs::Twist>("cmd_pos", 1, &TransporterNode::posCallback,this);
  joyvel_sub = nh.subscribe<geometry_msgs::Twist>("joycmd_vel", 1, &TransporterNode::joyCallback,this);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  js_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  reset_odom_srv = nh.advertiseService("reset_odom", &TransporterNode::resetOdom, this);

  ph.param("cov_x",cov_x, 1e-3);
  ph.param("cov_y",cov_y, 1e-3);
  ph.param("cov_th",cov_th, 1e-3);
  ph.param("cov_vx",cov_vx, 1e-3);
  ph.param("cov_vy",cov_vy, 1e-3);
  ph.param("cov_vth",cov_vth, 1e-3);

  ph.getParam("LEFT_MOTOR",trobot.LEFT_MOTOR);
  ph.getParam("RIGHT_MOTOR",trobot.RIGHT_MOTOR);

  ph.getParam("AXLE_LEN",trobot.AXLE_LEN);
  ph.getParam("WHEEL_DIA",trobot.WHEEL_DIA);
  ph.getParam("GEAR_RATIO",trobot.GEAR_RATIO);
  ph.getParam("STEPS_PER_REV",trobot.STEPS_PER_REV);
  ph.getParam("wheel_circum_correction",trobot.wheel_circum_correction);
  ph.getParam("wheel_base_correction",trobot.wheel_base_correction);
  ph.getParam("odom_angular_scale_correction",trobot.odom_angular_scale_correction);

  ph.getParam("CurrentRegulatorPGain",trobot.CurrentRegulatorPGain);
  ph.getParam("CurrentRegulatorIGain",trobot.CurrentRegulatorIGain);
  ph.getParam("VelocityPGain",trobot.VelocityPGain);
  ph.getParam("VelocityIGain",trobot.VelocityIGain);
  ph.getParam("PositionPGain",trobot.PositionPGain);
  ph.getParam("PositionIGain",trobot.PositionIGain);
  ph.getParam("PositionDGain",trobot.PositionDGain);

  ph.getParam("ProfileAcceleration",trobot.ProfileAcceleration);
  ph.getParam("ProfileDeceleration",trobot.ProfileDeceleration);
  ph.getParam("ProfileVelocity",trobot.ProfileVelocity);
  ph.getParam("MaxProfileVelocity",trobot.MaxProfileVelocity);
  ph.getParam("MaxFollowError",trobot.MaxFollowError);
  ph.getParam("PositionProfileWindow",trobot.PositionProfileWindow);
  ph.getParam("PositionProfileWindowTime",trobot.PositionProfileWindowTime);

  ph.getParam("MotorMaxContinuousCurrent",trobot.MotorMaxContinuousCurrent);
  ph.getParam("ThermalTimeConstantWinding",trobot.ThermalTimeConstantWinding);

  current_time = ros::Time::now();
  last_time = ros::Time::now();
}

void TransporterNode::estopCallback(const std_msgs::Bool::ConstPtr& msg)
{
	ROS_ERROR("estopped");
	estopped = msg->data;
}

void TransporterNode::sensorCallback(const std_msgs::Bool::ConstPtr& msg)
{
	sensor = msg->data;
}

void TransporterNode::restartRobot()
{
  int tries = 0;

  if (!resetting)
  	resetting = true;
  else return;
  
  while(ros::ok() && !trobot.reset())
  {
	ROS_ERROR("Failed to contact EPOS, going to try restarting EPOS, tries = %i...",tries);
	sleep(1);
	tries++;
	if (tries>10)
	{
		tries = 0;
		trobot.shutDown();
		trobot.start();
	}
  }
  resetting = errorState = false;
}

bool TransporterNode::resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  cur_theta = 0;
  cur_x = 0;
  cur_y = 0; 
  return true;
}

void TransporterNode::joyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  joylinear_ = msg->linear.x;
  joyangular_= msg->angular.z;
}

void TransporterNode::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  linear_ = msg->linear.x;
  angular_= msg->angular.z;
  
  if(sensor)
  {
  	sensor = false; //resetting it in case sensor callback fails for whatever reason, robot won't be stuck in turning
  	angular_-=0.5;
  }

  ROS_INFO("Move at vx [%f] & w [%f]", linear_, angular_);
}

void TransporterNode::posCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Move dist [%f] & angle [%f]", msg->linear.x, msg->angular.z);
//  trobot.moveDistance(msg->linear.x);
//  trobot.moveAngle(msg->angular.z);
//  publish_odom();
}

void TransporterNode::publish_odom()
{
  if(errorState) return;
  
// store the current pose
  prevPose.x = cur_x;
  prevPose.y = cur_y;
  prevPose.theta = cur_theta;

// calculate the new pose
  double distance, angle;
  
  errorState = !trobot.getDisplacement(&distance, &angle);
  if(errorState)
  {
	ROS_ERROR("getDisplacement failed in errorState.");
	return;
  }
  
  cur_theta += angle;
  cur_x += distance * cos(cur_theta);
  cur_y += distance * sin(cur_theta); 
  ROS_DEBUG("PREVIOUS x = [%f], y = [%f], theta = [%f]\n", prevPose.x, prevPose.y, prevPose.theta);
  ROS_DEBUG("CURRENT x = [%f], y = [%f], theta = [%f]\n", cur_x, cur_y, cur_theta);

  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();

  // convert rotation about z into quaternion
  geometry_msgs::Quaternion odom_quat;
  odom_quat.z = sin(cur_theta/2.0);
  odom_quat.w = cos(cur_theta/2.0);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  //set the position (with respect to header.frame)
  odom.pose.pose.position.x = cur_x;
  odom.pose.pose.position.y = cur_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance = 
		boost::array<double, 36>{{cov_x, 0, 0, 0, 0, 0, 
								0, cov_y, 0, 0, 0, 0,
								0, 0, 1e9, 0, 0, 0,
								0, 0, 0, 1e9, 0, 0,
								0, 0, 0, 0, 1e9, 0,
								0, 0, 0, 0, 0, cov_th}};

  //set the velocity (with respect to child_frame)
  odom.twist.twist.linear.x = distance/dt;
  odom.twist.twist.angular.z = angle/dt;
  odom.twist.covariance = 
		boost::array<double, 36>{{cov_vx, 0, 0, 0, 0, 0, 
								0, cov_vy, 0, 0, 0, 0,
								0, 0, 1e9, 0, 0, 0,
								0, 0, 0, 1e9, 0, 0,
								0, 0, 0, 0, 1e9, 0,
								0, 0, 0, 0, 0, cov_vth}};

  //publish the message
  odom_pub.publish(odom);

// publish joint state in order to show the continuous joints (wheels) in tf
  sensor_msgs::JointState js;
  js.header.stamp=current_time;
  js.name.resize(2);
  js.position.resize(2);
  js.name[0]="left_wheel_joint";
  js.name[1]="right_wheel_joint";
  js.position[0]=0;
  js.position[1]=0;
  js_pub.publish(js);

  last_time = current_time;
} //void TransporterNode::publish_odom()

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transporter");
  TransporterNode tNode;
  ros::NodeHandle n;
  double rate = 10;
  n.getParam("odom_pub_rate", rate);
  ros::Rate r(rate);

  tNode.trobot.start();
  while(ros::ok() && !tNode.trobot.reset())
  {
	ROS_WARN("EPOS not ready. Trying again in 1 second....");
	sleep(1);
  }
  ROS_INFO("Robot waiting for command...");

  while(ros::ok())
  {
	tNode.publish_odom();
	if(tNode.errorState)
	{
		tNode.restartRobot();
		continue;
	}

	if (!tNode.estopped)
		tNode.errorState = !tNode.trobot.moveVelocity(tNode.linear_,tNode.angular_);
	else
		tNode.errorState = !tNode.trobot.moveVelocity(tNode.joylinear_,tNode.joyangular_);
	if(tNode.errorState) continue;
	
	ros::spinOnce();
	r.sleep();
  }
  return 0;
}
