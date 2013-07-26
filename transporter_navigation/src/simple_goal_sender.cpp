#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ClientNode
{
private:
	MoveBaseClient ac;
	bool sendNextGoal;
	ros::NodeHandle n;
	std::string goal_frame_;

public:
struct GoalPose {
	double goal_x, goal_y, goal_theta;
};
	std::vector<GoalPose> targets;
	std::vector<GoalPose>::iterator target_it;
	ClientNode():
		 ac("move_base", true), sendNextGoal(true), n("~")
	{
		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for the move_base action server to come up");
		n.getParam("goal_frame", goal_frame_);
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
				goal.target_pose.pose.position.x = target_it->goal_x;
				goal.target_pose.pose.position.y = target_it->goal_y;
				goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_it->goal_theta);
				ROS_INFO("Sending goal (%lf, %lf, %lf)",target_it->goal_x,target_it->goal_y,target_it->goal_theta);
				ac.sendGoal(goal,
					boost::bind(&ClientNode::doneCb, this, _1, _2),
					MoveBaseClient::SimpleActiveCallback(),
					boost::bind(&ClientNode::feedbackCb, this, _1));
				sendNextGoal = false;
				target_it++;
			}
			ros::spinOnce();
		}
	}

	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		if (target_it==targets.end())
			ros::shutdown();
		else
			sendNextGoal = true;		
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
//		double cur_x = feedback->base_position.pose.position.x;
//		ROS_INFO("Current x : %f", cur_x);
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
		fscanf(goalsFile, "Position(%lf, %lf) Orientation(%lf)\n", &goal.goal_x, &goal.goal_y,&goal.goal_theta);
		ROS_INFO("Got goal (x,y,theta)=(%lf, %lf,%lf)", goal.goal_x, goal.goal_y,goal.goal_theta);
		goal_client_node.targets.push_back(goal);
	}
  
	goal_client_node.start();
	return 0;
}

