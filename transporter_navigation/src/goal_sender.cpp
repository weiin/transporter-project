#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ClientNode
{
private:
	MoveBaseClient ac;
	bool absGoal, sendNextGoal, sendGPS, repeat_;
	ros::NodeHandle n;
	ros::Subscriber gps_sub;
	std::string goal_frame_;
	double GPSerr;

public:
	struct GoalPose {
		int numOfTgts;
		double gps_lon, gps_lat, utm_x, utm_y, pose_x, pose_y, pose_theta;
	};
	std::vector<GoalPose> targets;
	std::vector<GoalPose>::iterator target_it;
	GoalPose curPose;
	
	ClientNode():
		 ac("move_base", true), sendNextGoal(true), sendGPS(false), repeat_(false), n("~")
	{
		//wait for the action server to come up
	//	while(!ac.waitForServer(ros::Duration(5.0)))
	//		ROS_INFO("Waiting for the move_base action server to come up");
		n.param<std::string>("goal_frame", goal_frame_, "/map");
		gps_sub = n.subscribe<nav_msgs::Odometry>("/gps_odom", 1, &ClientNode::GPScallback,this);
	}
	
	void start()
	{
		move_base_msgs::MoveBaseGoal goal;
		target_it = targets.begin();

		while(ros::ok())
		{
			if (sendNextGoal)
			{
				goal.target_pose.header.frame_id = goal_frame_;
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = target_it->pose_x;
				goal.target_pose.pose.position.y = target_it->pose_y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_it->pose_theta);
				ROS_INFO("Sending goal (%lf, %lf, %lf)",target_it->pose_x,target_it->pose_y,target_it->pose_theta);
				ac.sendGoal(goal,
					boost::bind(&ClientNode::doneCb, this, _1, _2),
					MoveBaseClient::SimpleActiveCallback(),
					boost::bind(&ClientNode::feedbackCb, this, _1));
				sendNextGoal = false;
				sendGPS = false;
				absGoal = false;
			}
			if (sendGPS)
			{
				double dx = target_it->utm_x - curPose.utm_x;
				double dy = target_it->utm_y - curPose.utm_y;
				double cth = cos(curPose.pose_theta);
				double sth = sin(curPose.pose_theta);
				
				goal.target_pose.header.frame_id = "base_link";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = dx/cth + (dy - dx*sth/cth)*sth;
				goal.target_pose.pose.position.y = dy*cth - dx*sth;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

				ROS_INFO("Sending GPS (%lf, %lf, %lf)",target_it->utm_x,target_it->utm_y,target_it->pose_theta);
				ac.sendGoal(goal,
					boost::bind(&ClientNode::doneCb, this, _1, _2),
					MoveBaseClient::SimpleActiveCallback(),
					boost::bind(&ClientNode::feedbackCb, this, _1));
				sendNextGoal = false;
				sendGPS = false;
				absGoal = true;
			}
			ros::spinOnce();
		} //while (ros::ok())
	} //void start()

	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		
		bool useGPS = false;
		n.getParam("useGPSgoal", useGPS);
		if(useGPS)
		// goal with relative coords reached, but still have not reached reqd GPS coord
		if(!absGoal && distance(curPose.utm_x, curPose.utm_y, target_it->utm_x, target_it->utm_y) > GPSerr)
		{
			sendGPS = true;
			return;
		}
		
		target_it++; //current goal done, go to next goal
		if (target_it==targets.end())
		{
			n.getParam("repeat", repeat_);
			if(repeat_)
			{
				target_it = targets.begin();
				sendNextGoal = true;
			}
			else
			{
				ROS_INFO("Ended with E,N,theta :( %f, %f, %f )", curPose.utm_x,curPose.utm_y,curPose.pose_theta);
				ros::shutdown();
			}
		}
		else sendNextGoal = true;
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
		curPose.pose_x = feedback->base_position.pose.position.x;
		curPose.pose_y = feedback->base_position.pose.position.y;
		curPose.pose_theta = tf::getYaw(feedback->base_position.pose.orientation);
		//ROS_INFO("Current x,y,theta :( %f, %f, %f )", curPose.pose_x,curPose.pose_y,curPose.pose_theta);
	}
	
	void GPScallback(const nav_msgs::Odometry::ConstPtr& gpsOdom)
	{
		curPose.utm_x = gpsOdom->pose.pose.position.x;
		curPose.utm_y = gpsOdom->pose.pose.position.y;
		GPSerr = gpsOdom->pose.covariance[0]; //gps precision error is same for x and y (obtained from DOP)
		//ROS_INFO("Current GPS (E,N) err = (%f,%f) %f", curPose.utm_x, curPose.utm_y, GPSerr); 
	}
	
	double distance(double start_x, double start_y, double end_x, double end_y)
	{
		return sqrt((end_x-start_x)*(end_x-start_x)+(end_y-start_y)*(end_y-start_y));
	}
	
}; //class ClientNode

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_sender");
	ClientNode goal_client_node;
	ClientNode::GoalPose goal;

	ROS_INFO("Opening file %s...", argv[1]);
	FILE * goalsFile;
	goalsFile = fopen(argv[1], "r");
	if(goalsFile == NULL)
		ROS_FATAL("Cannot open file %s", argv[1]);

	while(!feof(goalsFile))
	{
		if(fscanf(goalsFile, "%d GPS(%lf, %lf) UTM(%lf, %lf) Position(%lf, %lf) Orientation(%lf)\n", 
						&goal.numOfTgts, &goal.gps_lon, &goal.gps_lat, &goal.utm_x, &goal.utm_y, &goal.pose_x, &goal.pose_y,&goal.pose_theta))
		{
			ROS_INFO("Got goal with [%d] targets: (x,y,theta)=(%.3lf, %.3lf, %.3lf), UTM=(%.3lf, %.3lf), GPS=(%.7lf, %.7lf)",
					goal.numOfTgts, goal.pose_x, goal.pose_y,goal.pose_theta, goal.utm_x, goal.utm_y, goal.gps_lon, goal.gps_lat);
			goal_client_node.targets.push_back(goal);
		}
	}
  
	goal_client_node.start();
	return 0;
}

