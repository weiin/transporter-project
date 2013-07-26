#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "techx_comms/RobotState.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ClientNode
{
private:
	MoveBaseClient ac;
	bool repeat_;
	ros::NodeHandle nh, ph;
	ros::Subscriber gps_sub, state_sub;
	ros::Publisher state_pub;
	std::string goal_frame_;
	double GPSerr;
	techx_comms::RobotState curr_state;

public:
	struct GoalPose {
		int numOfTgts;
		char tgt_type;
		double gps_lon, gps_lat, utm_x, utm_y, pose_x, pose_y, pose_theta;
	};
	std::vector<GoalPose> targets;
	std::vector<GoalPose>::iterator target_it;
	GoalPose curPose;
	
	ClientNode():
		 ac("move_base", true), repeat_(false), ph("~")
	{
		ph.param<std::string>("goal_frame", goal_frame_, "/base_link");
		gps_sub = nh.subscribe<nav_msgs::Odometry>("gps_odom", 1, &ClientNode::GPScallback,this);
		state_sub = nh.subscribe<techx_comms::RobotState>("state", 1, &ClientNode::stateCallback,this);
		state_pub = nh.advertise<techx_comms::RobotState>("state", 1);
		
		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	void start()
	{
		move_base_msgs::MoveBaseGoal goal;
		target_it = targets.begin();
		ros::Rate r(20);
		
		while(curr_state.header.stamp.isZero() || curr_state.state != techx_comms::RobotState::MOVING_WAYPOINT)
		{
			ros::spinOnce();
			ROS_WARN("GPS sender waiting for waypoint sender to send first waypoint");
			r.sleep();
		}
		
		// now update the state with target info
		curr_state.header.stamp = ros::Time::now();
		curr_state.header.frame_id = "gpspoint";
		curr_state.target_lat = target_it->gps_lat;
		curr_state.target_lon = target_it->gps_lon;
		curr_state.num_of_targets = target_it->numOfTgts;
		state_pub.publish(curr_state);

		while(ros::ok())
		{
			if (curr_state.state == techx_comms::RobotState::SENDNEXT_GPSPOINT)
			{
				bool useGPS = false;
				ph.getParam("useGPSgoal", useGPS);

				double dx = target_it->utm_x - curPose.utm_x;
				double dy = target_it->utm_y - curPose.utm_y;
				double cth = cos(curPose.pose_theta);
				double sth = sin(curPose.pose_theta);
				
				goal.target_pose.header.frame_id = goal_frame_;
				goal.target_pose.header.stamp = ros::Time::now();

				if(!useGPS || distance(curPose.utm_x, curPose.utm_y, target_it->utm_x, target_it->utm_y) < GPSerr)
				{
					goal.target_pose.pose.position.x = 0;
					goal.target_pose.pose.position.y = 0;
				} else
				{
					goal.target_pose.pose.position.x = dx/cth + (dy - dx*sth/cth)*sth;
					goal.target_pose.pose.position.y = dy*cth - dx*sth;
				}
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

				ROS_INFO("Sending GPS (%lf, %lf, %lf)",target_it->utm_x,target_it->utm_y,target_it->pose_theta);
				ac.sendGoal(goal,
					boost::bind(&ClientNode::doneCb, this, _1, _2),
					MoveBaseClient::SimpleActiveCallback(),
					boost::bind(&ClientNode::feedbackCb, this, _1));

				curr_state.header.stamp = ros::Time::now();
				curr_state.header.frame_id = "gpspoint";
				curr_state.state = techx_comms::RobotState::MOVING_GPSPOINT;
				curr_state.target_x = goal.target_pose.pose.position.x;
				curr_state.target_y = goal.target_pose.pose.position.y;
				state_pub.publish(curr_state);
			}
			ros::spinOnce();
			r.sleep();
		} //while (ros::ok())
	} //void start()

	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		if (curr_state.state != techx_comms::RobotState::MOVING_GPSPOINT)
			return; //this goal was not sent by this node, so ignore

		ROS_INFO("GPSpoint finished in state [%s]", state.toString().c_str());
		
		target_it++;
		if (target_it==targets.end())
		{
			ph.getParam("repeat", repeat_);
			if(repeat_)
				target_it = targets.begin();
			else
				ROS_INFO("GPSpoint ended with E,N,theta :( %f, %f, %f )", curPose.utm_x,curPose.utm_y,curPose.pose_theta);
		}

		curr_state.header.stamp = ros::Time::now();
		curr_state.header.frame_id = "gpspoint";
		if (target_it==targets.end())
		{
			curr_state.target_lat = -1.;
			curr_state.target_lon = -1.;
			curr_state.num_of_targets = 0;
		} else
		{
			curr_state.target_lat = target_it->gps_lat;
			curr_state.target_lon = target_it->gps_lon;
			curr_state.num_of_targets = target_it->numOfTgts;
		}
		curr_state.state = techx_comms::RobotState::SENDNEXT_WAYPOINT;
		state_pub.publish(curr_state);

		if (target_it==targets.end())
		{
			usleep(500000);
			ros::shutdown();
		}
		
	} // void doneCb

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
	
	void stateCallback(const techx_comms::RobotState::ConstPtr& state)
	{
		curr_state = *state;
	}
	
	double distance(double start_x, double start_y, double end_x, double end_y)
	{
		return sqrt((end_x-start_x)*(end_x-start_x)+(end_y-start_y)*(end_y-start_y));
	}
	
}; //class ClientNode

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gps_sender");
	ClientNode goal_client_node;
	ClientNode::GoalPose goal;

	ROS_INFO("Opening file %s...", argv[1]);
	FILE * goalsFile;
	goalsFile = fopen(argv[1], "r");
	if(goalsFile == NULL)
	{
		ROS_FATAL("Cannot open file %s", argv[1]);
		return -1;
	}

	// read in first line
	fscanf(goalsFile, "NumofTgts # TgtType O/B/S GPS(lon[deg],lat[deg]) UTM(lon[m],lat[m]) Pose2D(x[m],y[m],theta[rad])\n");

	while(!feof(goalsFile))
	{
		if(fscanf(goalsFile, "NumofTgts %d TgtType %c GPS(%lf, %lf) UTM(%lf, %lf) Pose2D(%lf, %lf,%lf)\n", 
						&goal.numOfTgts, &goal.tgt_type, &goal.gps_lon, &goal.gps_lat, &goal.utm_x, &goal.utm_y, &goal.pose_x, &goal.pose_y,&goal.pose_theta))
		{
			if (goal.numOfTgts > 0)
			{
				ROS_INFO("Read [%d] target-type [%c]: (x,y,theta)=(%.3lf, %.3lf, %.3lf), UTM=(%.3lf, %.3lf), GPS=(%.7lf, %.7lf)", goal.numOfTgts, goal.tgt_type, goal.pose_x, goal.pose_y,goal.pose_theta, goal.utm_x, goal.utm_y, goal.gps_lon, goal.gps_lat);
				goal_client_node.targets.push_back(goal);
			}
		}
	}
  
	goal_client_node.start();
	return 0;
}
