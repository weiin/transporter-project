// convert robot pose data from tf to msg

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

int main( int argc, char** argv)
{
	ros::init(argc, argv, "pose_publisher");

	ros::NodeHandle n;
	ros::Publisher pose_pub = n.advertise<geometry_msgs::TransformStamped>("robot_pose", 20);
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped robotpose;

	ros::Rate r(1);
	while (n.ok())
	{
		ros::spinOnce();
		r.sleep();
		try
		{
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			tf::transformStampedTFToMsg(transform,robotpose);
		}
		catch (tf::TransformException ex)
		{
			ROS_DEBUG("%s",ex.what());
			continue;
		}

		pose_pub.publish(robotpose);
	}
} //main
