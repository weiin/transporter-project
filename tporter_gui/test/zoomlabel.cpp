//#include "../include/tporter_gui/zoomlabel.h"
//#include "../include/tporter_gui/main_window.hpp"
using namespace tporter_gui;

zoomLabel::zoomLabel(QWidget *parent)
	: QLabel(parent)
{
}

void zoomLabel::wheelEvent( QWheelEvent* event)
{
//	if(!ui.map_label->hasFocus()) return;
	float f;
	if (event->modifiers().testFlag(Qt::ControlModifier)) { // zoom only when CTRL key pressed
		int numSteps = event->delta() / 15 / 8;
 		if (numSteps == 0) {
			event->ignore();
			return;
		}
		else if (numSteps == 1)
			f = 1.25;
		else if (numSteps == -1)
			f = 0.8;
		scaleImage(f);
		event->accept();
	}
}

