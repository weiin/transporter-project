#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <math.h>
#include "techx_comms/RobotState.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ClientNode
{
private:
	MoveBaseClient ac;
	bool repeat_;
	ros::NodeHandle nh, ph;
	ros::Subscriber state_sub;
	ros::Publisher state_pub;
	std::string goal_frame_;
	techx_comms::RobotState curr_state;
	ros::Time last_sent;

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
		ph.param<std::string>("goal_frame", goal_frame_, "/map");

		state_sub = nh.subscribe<techx_comms::RobotState>("state", 1, &ClientNode::stateCallback,this);
		state_pub = nh.advertise<techx_comms::RobotState>("state", 1);

		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	void start()
	{
		move_base_msgs::MoveBaseGoal goal;
		target_it = targets.begin()-1;
		ros::Rate r(20);
		last_sent = ros::Time::now();

		while(ros::ok())
		{
			if (curr_state.state == techx_comms::RobotState::SENDNEXT_WAYPOINT)
			{
				if(ros::Time::now()-last_sent < ros::Duration(0.2)) continue;
				
				target_it++;
				if (target_it==targets.end())
				{
					ph.getParam("repeat", repeat_);
					if(repeat_)
						target_it = targets.begin();
					else
						ROS_INFO("Waypoint ended with x,y,theta :( %f, %f, %f )", curPose.pose_x,curPose.pose_y,curPose.pose_theta);
				}

				curr_state.header.stamp = ros::Time::now();
				curr_state.header.frame_id = "waypoint";
				if (target_it==targets.end())
					curr_state.state = techx_comms::RobotState::TASK_COMPLETE;
				else
				{
					curr_state.state = techx_comms::RobotState::MOVING_WAYPOINT;
					curr_state.target_x = target_it->pose_x;
					curr_state.target_y = target_it->pose_y;
				}
				state_pub.publish(curr_state);

				if (target_it==targets.end())
					ros::shutdown();

				goal.target_pose.header.frame_id = goal_frame_;
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = target_it->pose_x;
				goal.target_pose.pose.position.y = target_it->pose_y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_it->pose_theta);
				ROS_INFO("Sending waypoint (%lf, %lf, %lf)",target_it->pose_x,target_it->pose_y,target_it->pose_theta);
				ac.sendGoal(goal,
					boost::bind(&ClientNode::doneCb, this, _1, _2),
					MoveBaseClient::SimpleActiveCallback(),
					boost::bind(&ClientNode::feedbackCb, this, _1));
				last_sent = goal.target_pose.header.stamp;

			}

			ros::spinOnce();
			r.sleep();
		} //while (ros::ok())
	} //void start()

	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		if (curr_state.state != techx_comms::RobotState::MOVING_WAYPOINT)
			return; //this goal was not sent by this node, so ignore

		ROS_INFO("Waypoint completed in state [%s]", state.toString().c_str());

		curr_state.header.stamp = ros::Time::now();
		curr_state.header.frame_id = "waypoint";
		if (target_it->numOfTgts > 0) // goal has objects to find, make gps correction
		{
			curr_state.state = techx_comms::RobotState::SENDNEXT_GPSPOINT;
			state_pub.publish(curr_state);
			return;
		}

		curr_state.state = techx_comms::RobotState::SENDNEXT_WAYPOINT;
		state_pub.publish(curr_state);
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
		curPose.pose_x = feedback->base_position.pose.position.x;
		curPose.pose_y = feedback->base_position.pose.position.y;
		curPose.pose_theta = tf::getYaw(feedback->base_position.pose.orientation);
		//ROS_INFO("Current x,y,theta :( %f, %f, %f )", curPose.pose_x,curPose.pose_y,curPose.pose_theta);
	}
	
	void stateCallback(const techx_comms::RobotState::ConstPtr& state)
	{
		curr_state = *state;
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
	{
		ROS_FATAL("Cannot open file %s", argv[1]);
		return -1;
	}

	fscanf(goalsFile, "NumofTgts # TgtType O/B/S GPS(lon[deg],lat[deg]) UTM(lon[m],lat[m]) Pose2D(x[m],y[m],theta[rad])\n");
	while(!feof(goalsFile))
	{
		if(fscanf(goalsFile, "NumofTgts %d TgtType %c GPS(%lf, %lf) UTM(%lf, %lf) Pose2D(%lf, %lf,%lf)\n", 
						&goal.numOfTgts, &goal.tgt_type, &goal.gps_lon, &goal.gps_lat, &goal.utm_x, &goal.utm_y, &goal.pose_x, &goal.pose_y,&goal.pose_theta))
		{
			ROS_INFO("Read [%d] target-type [%c]: (x,y,theta)=(%.3lf, %.3lf, %.3lf), UTM=(%.3lf, %.3lf), GPS=(%.7lf, %.7lf)", goal.numOfTgts, goal.tgt_type, goal.pose_x, goal.pose_y,goal.pose_theta, goal.utm_x, goal.utm_y, goal.gps_lon, goal.gps_lat);
			goal_client_node.targets.push_back(goal);
		}
	}
  
	goal_client_node.start();
	return 0;
}
