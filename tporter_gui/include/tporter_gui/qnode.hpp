/**
 * @file /include/tporter_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tporter_gui_QNODE_HPP_
#define tporter_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QtGui>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tporter_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	void publishGoal(double x, double y, double theta);
	void setPose(QPoint initPose);
	void publishVel(double angular, double linear);
	void publishEStop(bool estopped);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void cmdvel(double, double);
    void robotPose(double, double, QImage);
	void loadMap(QImage);
	void goalStatus(bool);
	
private:
	int init_argc;
	char** init_argv;
	ros::Subscriber vel_subscriber, pose_subscriber, status_subscriber;
	ros::ServiceClient map_client;
	ros::Publisher goal_publisher, initPose_publisher, vel_pub, estop_pub;
    QStringListModel logging_model;

	QImage overlayImage;
   	QImage occupancy_map;
	QPoint map_origin;
	double map_resolution;

	void velCallback(const geometry_msgs::Twist::ConstPtr &msg);
	void statusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);
	void drawRobotPos(tf::StampedTransform transform);
	void drawMap(nav_msgs::OccupancyGrid map);
};

}  // namespace tporter_gui

#endif /* tporter_gui_QNODE_HPP_ */
