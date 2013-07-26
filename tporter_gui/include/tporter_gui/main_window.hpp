/**
 * @file /include/tporter_gui/main_window.hpp
 *
 * @brief Qt based gui for tporter_gui.
 *
 * @date November 2010
 **/
#ifndef tporter_gui_MAIN_WINDOW_H
#define tporter_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <nav_msgs/OccupancyGrid.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace tporter_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	void on_actionAbout_triggered();
	void on_button_connect_clicked();
	void on_button_go_clicked();
	void on_button_add_clicked();
	void on_button_del_clicked();
	void on_button_estop_clicked(bool checked);

    void updateLoggingView();
	void displayVel(double linear, double angular);
	void drawMap(QImage occupancy_map);
	void drawRobotPos(double x, double y, QImage robotPosMap);
	void goal_queue(bool);
Q_SIGNALS:
	void continueGoal();

protected:
	void wheelEvent( QWheelEvent* );
	void mousePressEvent ( QMouseEvent * event );
	void mouseMoveEvent ( QMouseEvent * event );
	void mouseReleaseEvent ( QMouseEvent * event );

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	double scaleFactor, fitScale;
	QPoint lastPoint; // holds position of cursor at last mouse button click
	bool bMovingImage;
	QMap<QString, QPoint> locations;

	void adjustScrollBar(QScrollBar *scrollBar, double factor);
	void scaleImage(double factor);
	void resizeWindow();
	void populateLocations();
};

}  // namespace tporter_gui

#endif // tporter_gui_MAIN_WINDOW_H
