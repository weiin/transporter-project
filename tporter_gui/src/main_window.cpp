/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <algorithm>
#include "../include/tporter_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tporter_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging and displays
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    QObject::connect(&qnode, SIGNAL(cmdvel(double, double)), this, SLOT(displayVel(double, double)));
    QObject::connect(&qnode, SIGNAL(robotPose(double, double, QImage)), this, SLOT(drawRobotPos(double, double, QImage)));
    QObject::connect(&qnode, SIGNAL(loadMap(QImage)), this, SLOT(drawMap(QImage)));
    QObject::connect(&qnode, SIGNAL(goalStatus(bool)), this, SLOT(goal_queue(bool)));

    QObject::connect(this, SIGNAL(continueGoal()), this, SLOT(on_button_go_clicked()));

	// set map_label as child of scrollArea in order to make scrollbars appear when image is bigger than area
	QWidget *p = ui.scrollArea->takeWidget();
	ui.scrollArea->setWidget(ui.map_label);
	delete p;
	resizeWindow();
	populateLocations();

}

MainWindow::~MainWindow() {}

void MainWindow::resizeWindow()
{
	QDesktopWidget *desktop = QApplication::desktop();
	int w, h;
	w = desktop->width();
	h = desktop->height();
	resize(w,h);
	ui.scrollArea->resize(w-ui.command_groupBox->width()-100,h*0.7);
	ui.command_groupBox->move(ui.scrollArea->x()+ui.scrollArea->width()+30,ui.scrollArea->y());
	ui.log_groupBox->resize(w*.9, h*0.75);
}

void MainWindow::populateLocations()
{
	locations["Home"]=QPoint(0, 0);
	locations["Lift"]=QPoint(6.8, 2.89);
	locations["Pillar"]=QPoint(3.78, 3.71);
	locations["TwoTwo"]=QPoint(2, 2);
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
}

void MainWindow::on_button_connect_clicked() {
	if ( !qnode.init() ) {
		showNoMasterMessage();
	} else {
		ui.button_connect->setEnabled(false);
		ui.setInitPose->setEnabled(true);
		ui.location_combo->setEnabled(true);
		ui.button_add->setEnabled(true);
	}
}

void MainWindow::on_button_go_clicked() {
	QPoint goal;
	QString goal_name = ui.goals_list->item(0)->text();
	goal = locations.value(goal_name);
	qnode.publishGoal(goal.x(),goal.y(),0);

	ui.statusbar->showMessage( "Going to " + goal_name );
	ui.current_goal->setText(goal_name);

	ui.goals_list->takeItem(0);
	ui.button_go->setEnabled(false);

	if (ui.goals_list->count()==0)
	{
		ui.button_del->setEnabled(false);
	}
}

void MainWindow::on_button_add_clicked()
{
	QString prevText;
	if (ui.goals_list->count()>0)
		prevText = ui.goals_list->item(ui.goals_list->count()-1)->text();

	// add goal to queue only if different from previous
	if(prevText != ui.location_combo->currentText())
	{
		ui.goals_list->addItem(ui.location_combo->currentText());
		ui.button_go->setEnabled(!ui.button_estop->isChecked());
		ui.button_del->setEnabled(true);
	}
}

void MainWindow::on_button_del_clicked()
{
	ui.goals_list->takeItem(ui.goals_list->currentRow());

	if (ui.goals_list->count()==0)
	{
		ui.button_del->setEnabled(false);
		ui.button_go->setEnabled(false);
	}
}

void MainWindow::on_button_estop_clicked(bool checked)
{
	qnode.publishEStop(checked);
	ui.button_go->setEnabled(!checked);
	if (checked) ui.button_estop->setText("&Push to release");
	else ui.button_estop->setText("&Emergency Stop");
}

void MainWindow::goal_queue(bool success)
{
	QString prev_goal = ui.current_goal->toPlainText();
	ui.current_goal->setText("");
	if (success)
	{
		ui.statusbar->showMessage("Reached "+prev_goal+" safely",2000);
		if (ui.goals_list->count()>0)
		{
			int ret = QMessageBox::question(this, "Goal reached",
									"Robot has reached the goal.\n"
									"Do you want robot to continue to next location in the goals list?",
	                                QMessageBox::Yes | QMessageBox::No,
									QMessageBox::Yes);
			if (ret==QMessageBox::Yes)
				Q_EMIT continueGoal();
			else
				ui.button_go->setEnabled(true);
		}
	}
	else
	{
		ui.statusbar->showMessage("Failed to reach "+prev_goal, 2000);
	}
}

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
	ui.view_logging->scrollToBottom();
}

void MainWindow::displayVel(double linear, double angular) {
	ui.robotLinear->display(linear);
	ui.robotAngular->display(angular);
}

void MainWindow::drawMap(QImage occupancy_map)
{
	ui.map_label->setPixmap(QPixmap::fromImage(occupancy_map));
	scaleFactor = 1.0;
	ui.map_label->adjustSize(); // map original size

	ui.statusbar->showMessage( QString("Loaded map - Width ")+QString::number(occupancy_map.width())
	 + QString(" x height ")+QString::number(occupancy_map.height()),2000);

	// fit image to map display area
	fitScale = std::min( (ui.scrollArea->height()-2) / (double)occupancy_map.height(),
						 (ui.scrollArea->width()-2) /(double)occupancy_map.width());
	scaleImage(fitScale);
}

void MainWindow::drawRobotPos(double x, double y, QImage robotPosMap)
{
	ui.map_label->setPixmap(QPixmap::fromImage(robotPosMap));
	ui.robotX->display(x);
	ui.robotY->display(y);
}

void MainWindow::adjustScrollBar(QScrollBar *scrollBar, double factor)
{
	// shifts scroll bar to keep it in the centre according to the scale factor
	scrollBar->setValue(int(factor * scrollBar->value() + ((factor - 1) * scrollBar->pageStep()/2)));
}

void MainWindow::scaleImage(double factor)
{
	if(!ui.map_label->pixmap()) return; // ensure there is image loaded
	scaleFactor *= factor;
	
	if(scaleFactor > 5.0 * fitScale || scaleFactor < fitScale) { // prevent over zooming
		scaleFactor /= factor;
		return;
	}
	ui.map_label->resize(scaleFactor * ui.map_label->pixmap()->size());

	adjustScrollBar(ui.scrollArea->horizontalScrollBar(), factor);
	adjustScrollBar(ui.scrollArea->verticalScrollBar(), factor);
}

void MainWindow::wheelEvent( QWheelEvent* event)
{
	if( !(ui.map_label->pixmap() && ui.map_label->underMouse()) ) {
		event->ignore();
		return;
	}
	
	if (event->modifiers().testFlag(Qt::ControlModifier)) { // zoom only when CTRL key pressed
		int numSteps = event->delta() / 15 / 8;
 		if (numSteps == 0) {
			event->ignore();
			return;
		}
		else if (numSteps == 1)
			scaleImage(1.25);
		else if (numSteps == -1)
			scaleImage(0.8);
		event->accept();
	}
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton) {
		lastPoint = event->pos();
		// if mouse click is within image, user wants to move image
		bMovingImage = (ui.map_label->pixmap() && ui.map_label->underMouse());
	}

	if(bMovingImage) setCursor(Qt::ClosedHandCursor);

	ui.statusbar->showMessage(QString("x: ")+QString::number(lastPoint.x())+QString(" y: ")+QString::number(lastPoint.y()) , 500);

	if (ui.setInitPose->isChecked())
	{
		QPoint tmp = ui.map_label->mapFromGlobal(mapToGlobal(lastPoint)) / scaleFactor; // mouse click pos in the map_label's coordinates
		qnode.setPose(tmp);
		ui.setInitPose->setChecked(false);
	}
}

void MainWindow::mouseMoveEvent ( QMouseEvent * event ) 
{
	if ((event->buttons() & Qt::LeftButton) && bMovingImage) {
		ui.scrollArea->horizontalScrollBar()->setValue( 
			ui.scrollArea->horizontalScrollBar()->value() - event->pos().x() + lastPoint.x() );
		ui.scrollArea->verticalScrollBar()->setValue( 
			ui.scrollArea->verticalScrollBar()->value() - event->pos().y() + lastPoint.y() );
		lastPoint = event->pos();
	}

	ui.statusbar->showMessage(QString("Moving... x: ")+QString::number(lastPoint.x())+QString(" y: ")+QString::number(lastPoint.y()) , 500);
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton && bMovingImage) {
		bMovingImage=false;
		setCursor(Qt::ArrowCursor);
		lastPoint = event->pos();
	}

	ui.statusbar->showMessage(QString("x: ")+QString::number(lastPoint.x())+QString(" y: ")+QString::number(lastPoint.y()) , 500);
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace tporter_gui

