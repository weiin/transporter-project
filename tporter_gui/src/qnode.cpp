/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include "../include/tporter_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tporter_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
{
	overlayImage = QImage(":/images/robot.png");
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
 	ros::init(init_argc,init_argv,"tporter_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	ros::NodeHandle n;
	// Add your ros communications here.
	vel_subscriber = n.subscribe("cmd_vel", 1, &QNode::velCallback, this);
	status_subscriber = n.subscribe("move_base/result", 1, &QNode::statusCallback, this);

	map_client = n.serviceClient<nav_msgs::GetMap>("static_map");

	goal_publisher = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	initPose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	estop_pub = n.advertise<std_msgs::Bool>("cmd_estop", 1);

	nav_msgs::GetMap srv;
	if ( map_client.call(srv) )
	{
		std::stringstream ss;
		ss << "Loaded map: Width [" << srv.response.map.info.width << "]";
		ss << " x Height [" << srv.response.map.info.height << "]";
		ss << " at Resolution [" << srv.response.map.info.resolution << "] m/pix";
		ss << "\nOrigin ("<<srv.response.map.info.origin.position.x<<","<<srv.response.map.info.origin.position.y<<")";
		log(Info, ss.str());

		drawMap(srv.response.map);
	}
	else
	{
		log(Warn, "Map request failed" );
	}

	start(); // starts the QNode thread QNode::run()
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	while ( ros::ok() ) {
		try{
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			drawRobotPos( transform );
		}
		catch (tf::TransformException ex){
			//ROS_ERROR("%s",ex.what());
		}
/*/TODO: make this estop a low level process instead of this workaround
			publishVel(0,0);
			publishGoal(transform.getOrigin().x(),transform.getOrigin().y(), tf::getYaw(transform.getRotation()) );
*/		
		ros::spinOnce();
		loop_rate.sleep();
	}

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::publishGoal(double x, double y, double theta)
{
	geometry_msgs::PoseStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "map";

	msg.pose.position.x = x;
	msg.pose.position.y = y;
	msg.pose.position.z = 0;
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = sin(theta/2.0);
	msg.pose.orientation.w = cos(theta/2.0);

	goal_publisher.publish(msg);
}

void QNode::setPose(QPoint initPose)
{
	// convert to actual map coordinates
	double x, y;
	x = (initPose.x() - map_origin.x()) * map_resolution;
	y = (occupancy_map.height()-initPose.y() - map_origin.y()) * map_resolution;

	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "map";
	msg.pose.pose.position.x = x;
	msg.pose.pose.position.y = y;
	msg.pose.pose.position.z = 0;
	msg.pose.pose.orientation.x = 0;
	msg.pose.pose.orientation.y = 0;
	msg.pose.pose.orientation.z = 0;
	msg.pose.pose.orientation.w = 1;

	initPose_publisher.publish(msg);
}
void QNode::publishEStop(bool estopped)
{
	std_msgs::Bool msg;
	msg.data = estopped;
	estop_pub.publish(msg);
	if (estopped)
		ROS_ERROR ("estopped");
}

void QNode::publishVel(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);    
}

void QNode::velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	log(Debug, "Linear.x = " + boost::lexical_cast<std::string>(msg->linear.x)
			+ " Angular.z = " + boost::lexical_cast<std::string>(msg->angular.z) );
	Q_EMIT cmdvel(msg->linear.x, msg->angular.z);
}

void QNode::statusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
	bool success=false;
	log(Debug, "Status = " + msg->status.text );
	if (msg->status.text=="Goal reached.")
		success = true;

	Q_EMIT goalStatus(success);
}
	
void QNode::drawRobotPos(tf::StampedTransform transform)
{
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
//	double theta = tf::getYaw(transform.getRotation());

	log(Debug, "Robot pos x = " + boost::lexical_cast<std::string>(x)
			+ " Robot pos y = " + boost::lexical_cast<std::string>(y) );

	int px = x / map_resolution + map_origin.x();
	int py = y / map_resolution + map_origin.y();

	QImage robotPosMap = occupancy_map.convertToFormat(QImage::Format_RGB32,Qt::DiffuseDither);
	QPainter painter(&robotPosMap);

	px = px - overlayImage.width()/2;
	py = robotPosMap.height() - py - overlayImage.height()/2;

	painter.drawImage(px, py, overlayImage);

	Q_EMIT robotPose(x, y, robotPosMap);
}

void QNode::drawMap(nav_msgs::OccupancyGrid map) {
	map_resolution = map.info.resolution; 
	map_origin = QPoint(-map.info.origin.position.x/map_resolution,-map.info.origin.position.y/map_resolution);
 
	// Convert OccupancyGrid probabilities 0, 100, -1 to grayscale pixels
	occupancy_map = QImage( map.info.width, map.info.height, QImage::Format_Indexed8);
    occupancy_map.setColor( 0, qRgb(0, 0, 0) ); // black
    occupancy_map.setColor( 1, qRgb(254, 254, 254) ); // white
    occupancy_map.setColor( 2, qRgb(205, 205, 205) ); // gray
	
	for (int row=0; row<occupancy_map.height(); row++)
	{
		for (int col=0; col<occupancy_map.width(); col++)
		{
			int occ = map.data[row*occupancy_map.width()+col];
			// map_server map origin starts from bottom left
			// but QImage origin starts from top left
			switch(occ)
			{
				case 100: // completely occupied
					occupancy_map.setPixel(col,occupancy_map.height()-row-1,0);
					break;
				case 0: // competely free
					occupancy_map.setPixel(col,occupancy_map.height()-row-1,1);
					break;
				case -1: // unknown
				default: // partially occupied - currently not implemented by occupancy grid
					occupancy_map.setPixel(col,occupancy_map.height()-row-1,2);
					break;
			}
		}
	}

	Q_EMIT loadMap(occupancy_map);
}

void QNode::log( const LogLevel &level, const std::string &msg)
{
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace tporter_gui
