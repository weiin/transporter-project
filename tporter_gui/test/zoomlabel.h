#ifndef ZOOMLABEL_H
#define ZOOMLABEL_H

#include <QtGui>

class zoomLabel : public QLabel
{
	Q_OBJECT
	
public:
	zoomLabel(QWidget *parent=0);
	
protected:
  void wheelEvent( QWheelEvent* );

};

#endif
