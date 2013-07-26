/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created: Fri Mar 9 14:17:54 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/tporter_gui/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_tporter_gui__MainWindow[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      25,   24,   24,   24, 0x05,

 // slots: signature, parameters, type, tag, flags
      40,   24,   24,   24, 0x0a,
      67,   24,   24,   24, 0x0a,
      95,   24,   24,   24, 0x0a,
     118,   24,   24,   24, 0x0a,
     142,   24,   24,   24, 0x0a,
     174,  166,   24,   24, 0x0a,
     204,   24,   24,   24, 0x0a,
     239,  224,   24,   24, 0x0a,
     279,  265,   24,   24, 0x0a,
     311,  295,   24,   24, 0x0a,
     346,   24,   24,   24, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_tporter_gui__MainWindow[] = {
    "tporter_gui::MainWindow\0\0continueGoal()\0"
    "on_actionAbout_triggered()\0"
    "on_button_connect_clicked()\0"
    "on_button_go_clicked()\0on_button_add_clicked()\0"
    "on_button_del_clicked()\0checked\0"
    "on_button_estop_clicked(bool)\0"
    "updateLoggingView()\0linear,angular\0"
    "displayVel(double,double)\0occupancy_map\0"
    "drawMap(QImage)\0x,y,robotPosMap\0"
    "drawRobotPos(double,double,QImage)\0"
    "goal_queue(bool)\0"
};

const QMetaObject tporter_gui::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_tporter_gui__MainWindow,
      qt_meta_data_tporter_gui__MainWindow, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &tporter_gui::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *tporter_gui::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *tporter_gui::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_tporter_gui__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int tporter_gui::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: continueGoal(); break;
        case 1: on_actionAbout_triggered(); break;
        case 2: on_button_connect_clicked(); break;
        case 3: on_button_go_clicked(); break;
        case 4: on_button_add_clicked(); break;
        case 5: on_button_del_clicked(); break;
        case 6: on_button_estop_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: updateLoggingView(); break;
        case 8: displayVel((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 9: drawMap((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 10: drawRobotPos((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< QImage(*)>(_a[3]))); break;
        case 11: goal_queue((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void tporter_gui::MainWindow::continueGoal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
