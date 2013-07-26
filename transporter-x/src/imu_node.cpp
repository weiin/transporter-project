/*
 * Node for publishing imu data from Microstrain 3DM-GX1
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include "imu_3dmgx1.cpp"
#include "tf/transform_broadcaster.h"
class ImuNode
{
public:
	ImuNode(ros::NodeHandle nh);
	~ImuNode();
	sensor_msgs::Imu imu_data;
	IMU_3DMGX1 imu;
	IMU_3DMGX1::ComPortHandle comPort;
	IMU_3DMGX1::EulerAngles eulerAngles;
	IMU_3DMGX1::Quaternion quaternion;
	IMU_3DMGX1::Acceleration accel;
	IMU_3DMGX1::AngularRate angRate;

	string port;
	ros::Publisher imu_data_pub_;
	ros::Publisher is_calibrated_pub_;
	ros::ServiceServer calibrate_serv_;

	bool calibrated_;
	bool calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
	bool init();
	void publish_imu_data();
	void publish_is_calibrated();
	int errcnt;
private:
	ros::NodeHandle ph;
	ros::Publisher imu_data_pub;
	double orientation_stdev, ang_rate_stdev, accel_stdev;
};

ImuNode::ImuNode(ros::NodeHandle nh):
	errcnt(0), ph("~")
{
    ros::NodeHandle imu_node_handle(nh, "imu");

	imu_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data", 1);
	is_calibrated_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated", 1, true);
	calibrate_serv_ = imu_node_handle.advertiseService("calibrate", &ImuNode::calibrate, this);

	ph.getParam("port",port);
	ph.getParam("orientation_stdev",orientation_stdev);
	ph.getParam("ang_rate_stdev",ang_rate_stdev);
	ph.getParam("accel_stdev",accel_stdev);

	double linear_acceleration_covariance = accel_stdev*accel_stdev;
	double angular_velocity_covariance = ang_rate_stdev*ang_rate_stdev;
	double orientation_covariance = orientation_stdev*orientation_stdev;

    imu_data.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    imu_data.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    imu_data.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    imu_data.angular_velocity_covariance[0] = angular_velocity_covariance;
    imu_data.angular_velocity_covariance[4] = angular_velocity_covariance;
    imu_data.angular_velocity_covariance[8] = angular_velocity_covariance;
    
    imu_data.orientation_covariance[0] = orientation_covariance;
    imu_data.orientation_covariance[4] = orientation_covariance;
    imu_data.orientation_covariance[8] = orientation_covariance;

} // ImuNode class constructor

ImuNode::~ImuNode()
{
	imu.CloseComPort(comPort);
} // ImuNode class destructor

bool ImuNode::init()
{
	do {
		comPort = imu.OpenComPort(port.c_str());
		if(comPort < 0) 
		{
			ROS_WARN("Port not ready. Will try again...");
			sleep(1);
		}
	} while (comPort < 0);
	while (!imu.getVersion(comPort))
	{
		ROS_WARN("No IMU detected. Will keep trying...");
		sleep(1);
	}

	calibrated_ = imu.init(comPort);
	publish_is_calibrated();
	return calibrated_;
}

void ImuNode::publish_imu_data()
{
	// get gyro-stabilized quaternion, instantaneous acceleration and bias-compensated angular rate
	if(imu.ReadQuatVect(comPort, quaternion, accel, angRate))
	{
#ifdef DEBUG
		cout <<"IMU -- North-East-Down convention" << endl;
		cout <<"w:" << quaternion.w << "  x:" << quaternion.x << "  y:" << quaternion.y<< "  z:" << quaternion.z <<endl;
 		cout <<"accel x:" << accel.x << "  accel y:" << accel.y<< "  accel z:" << accel.z << endl;
		cout <<"angrate x:" << angRate.x << "  angRate y:" << angRate.y<< "  angRate z:" << angRate.z << endl;
#endif

	// imu 3dmgx1 reports using the North-East-Down (NED) convention, ROS REP103 uses East-North-Up (ENU)
	// see http://answers.ros.org/question/626/quaternion-from-imu-interpreted-incorrectly-by-ros

		imu_data.orientation.w = quaternion.w;
		imu_data.orientation.x = quaternion.x;
		imu_data.orientation.y = -quaternion.y;
		imu_data.orientation.z = -quaternion.z;

#ifdef DEBUG
	double yaw, pitch, roll;
	tf::Quaternion tmp_;
	tf::quaternionMsgToTF(imu_data.orientation, tmp_);
	btMatrix3x3(tmp_).getRPY(roll, pitch, yaw);
	cout <<"roll: " << roll << "  pitch: " << pitch<< "  yaw: " << yaw << endl;
#endif

		imu_data.linear_acceleration.x = -accel.x;
		imu_data.linear_acceleration.y = accel.y;
		imu_data.linear_acceleration.z = accel.z;

		imu_data.angular_velocity.x = angRate.x;
		imu_data.angular_velocity.y = -angRate.y;
		imu_data.angular_velocity.z = -angRate.z;

		if(errcnt>0) errcnt--;

		imu_data.header.stamp = ros::Time::now();
		imu_data.header.frame_id = "gyro_link";
		imu_data_pub_.publish(imu_data);
	}
	else errcnt++;

	// the imu occasionally slows transmission, resulting in data bytes being broken up
	// using the getVersion to do a "resync", unless there is a better way?
	if(errcnt>10)
	{
		cout << "IMU not responding correctly too frequently, resyncing..." << endl;
		while (!imu.getVersion(comPort));
	}
} // void ImuNode::publish_imu_data()

void ImuNode::publish_is_calibrated()
{
	std_msgs::Bool msg;
	msg.data = calibrated_;
	is_calibrated_pub_.publish(msg);
}

bool ImuNode::calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	calibrated_ = imu.init(comPort);
	publish_is_calibrated();
	return calibrated_;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu");
	ros::NodeHandle n;
	ImuNode imu_node(n);
	double rate = 20;
	n.getParam("imu_pub_rate", rate);
	ros::Rate r(rate);
	//ros::Rate r(20); // the higher the rate, the more frequent the imu will have error

	imu_node.init();

	while(ros::ok())
	{
		imu_node.publish_imu_data();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
