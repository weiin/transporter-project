/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created: Fri Mar 9 17:35:09 2012
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QListWidget>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_displays;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QLabel *map_label;
    QGroupBox *command_groupBox;
    QWidget *widget;
    QVBoxLayout *verticalLayout_5;
    QGridLayout *gridLayout;
    QPushButton *button_connect;
    QSpacerItem *horizontalSpacer;
    QPushButton *quit_button;
    QPushButton *setInitPose;
    QSpacerItem *verticalSpacer_3;
    QVBoxLayout *verticalLayout_2;
    QComboBox *location_combo;
    QPushButton *button_add;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_3;
    QLabel *goals_label;
    QListWidget *goals_list;
    QHBoxLayout *horizontalLayout;
    QPushButton *button_go;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *button_del;
    QSpacerItem *verticalSpacer_2;
    QVBoxLayout *verticalLayout_4;
    QLabel *current_goal_label;
    QTextEdit *current_goal;
    QSpacerItem *verticalSpacer_4;
    QPushButton *button_estop;
    QWidget *tab_logging;
    QGroupBox *log_groupBox;
    QVBoxLayout *verticalLayout;
    QListView *view_logging;
    QFrame *cmdvel_frame;
    QHBoxLayout *horizontalLayout_2;
    QLabel *robotSpeedLabel;
    QLabel *robotLinearLabel;
    QLCDNumber *robotLinear;
    QLabel *robotAngularLabel;
    QLCDNumber *robotAngular;
    QFrame *pose_frame;
    QHBoxLayout *horizontalLayout_3;
    QLabel *robotPoseLabel;
    QLabel *robotXLabel;
    QLCDNumber *robotX;
    QLabel *robotYLabel;
    QLCDNumber *robotY;
    QMenuBar *menubar;
    QMenu *menu_File;
    QMenu *menu_Help;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(941, 744);
        MainWindowDesign->setMinimumSize(QSize(800, 600));
        MainWindowDesign->setMouseTracking(false);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::C, QLocale::AnyCountry));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QString::fromUtf8("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::C, QLocale::AnyCountry));
        tab_manager->setUsesScrollButtons(false);
        tab_displays = new QWidget();
        tab_displays->setObjectName(QString::fromUtf8("tab_displays"));
        tab_displays->setMouseTracking(false);
        scrollArea = new QScrollArea(tab_displays);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setGeometry(QRect(21, 21, 612, 531));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(scrollArea->sizePolicy().hasHeightForWidth());
        scrollArea->setSizePolicy(sizePolicy);
        scrollArea->setFocusPolicy(Qt::NoFocus);
        scrollArea->setLocale(QLocale(QLocale::C, QLocale::AnyCountry));
        scrollArea->setFrameShape(QFrame::Panel);
        scrollArea->setFrameShadow(QFrame::Plain);
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scrollArea->setWidgetResizable(false);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 610, 449));
        map_label = new QLabel(scrollAreaWidgetContents);
        map_label->setObjectName(QString::fromUtf8("map_label"));
        map_label->setGeometry(QRect(0, 0, 131, 41));
        QSizePolicy sizePolicy1(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(map_label->sizePolicy().hasHeightForWidth());
        map_label->setSizePolicy(sizePolicy1);
        map_label->setFocusPolicy(Qt::WheelFocus);
        map_label->setAutoFillBackground(false);
        map_label->setScaledContents(true);
        scrollArea->setWidget(scrollAreaWidgetContents);
        command_groupBox = new QGroupBox(tab_displays);
        command_groupBox->setObjectName(QString::fromUtf8("command_groupBox"));
        command_groupBox->setGeometry(QRect(650, 20, 250, 534));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::MinimumExpanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(command_groupBox->sizePolicy().hasHeightForWidth());
        command_groupBox->setSizePolicy(sizePolicy2);
        widget = new QWidget(command_groupBox);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(4, 10, 241, 523));
        verticalLayout_5 = new QVBoxLayout(widget);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        button_connect = new QPushButton(widget);
        button_connect->setObjectName(QString::fromUtf8("button_connect"));
        button_connect->setEnabled(true);
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(button_connect->sizePolicy().hasHeightForWidth());
        button_connect->setSizePolicy(sizePolicy3);
        button_connect->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #fff, stop: 1 #888);\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}\n"
"QPushButton:!enabled{\n"
"color: #aaa;\n"
"border: 2px solid #aaa;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}"));

        gridLayout->addWidget(button_connect, 0, 0, 1, 1);

        horizontalSpacer = new QSpacerItem(20, 20, QSizePolicy::Maximum, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 0, 1, 1, 1);

        quit_button = new QPushButton(widget);
        quit_button->setObjectName(QString::fromUtf8("quit_button"));
        sizePolicy3.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy3);
        quit_button->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #fff, stop: 1 #888);\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}\n"
"QPushButton:!enabled{\n"
"color: #aaa;\n"
"border: 2px solid #aaa;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}"));

        gridLayout->addWidget(quit_button, 0, 2, 1, 1);

        setInitPose = new QPushButton(widget);
        setInitPose->setObjectName(QString::fromUtf8("setInitPose"));
        setInitPose->setEnabled(false);
        setInitPose->setCheckable(true);

        gridLayout->addWidget(setInitPose, 1, 0, 1, 3);


        verticalLayout_5->addLayout(gridLayout);

        verticalSpacer_3 = new QSpacerItem(233, 20, QSizePolicy::Minimum, QSizePolicy::Preferred);

        verticalLayout_5->addItem(verticalSpacer_3);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        location_combo = new QComboBox(widget);
        location_combo->setObjectName(QString::fromUtf8("location_combo"));
        location_combo->setEnabled(false);

        verticalLayout_2->addWidget(location_combo);

        button_add = new QPushButton(widget);
        button_add->setObjectName(QString::fromUtf8("button_add"));
        button_add->setEnabled(false);
        button_add->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #fff, stop: 1 #888);\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}\n"
"QPushButton:!enabled{\n"
"color: #aaa;\n"
"border: 2px solid #aaa;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}"));

        verticalLayout_2->addWidget(button_add);


        verticalLayout_5->addLayout(verticalLayout_2);

        verticalSpacer = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        goals_label = new QLabel(widget);
        goals_label->setObjectName(QString::fromUtf8("goals_label"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(goals_label->sizePolicy().hasHeightForWidth());
        goals_label->setSizePolicy(sizePolicy4);

        verticalLayout_3->addWidget(goals_label);

        goals_list = new QListWidget(widget);
        goals_list->setObjectName(QString::fromUtf8("goals_list"));
        goals_list->setMaximumSize(QSize(16777215, 150));

        verticalLayout_3->addWidget(goals_list);


        verticalLayout_5->addLayout(verticalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        button_go = new QPushButton(widget);
        button_go->setObjectName(QString::fromUtf8("button_go"));
        button_go->setEnabled(false);
        sizePolicy3.setHeightForWidth(button_go->sizePolicy().hasHeightForWidth());
        button_go->setSizePolicy(sizePolicy3);
        button_go->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #fff, stop: 1 #888);\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}\n"
"QPushButton:!enabled{\n"
"color: #aaa;\n"
"border: 2px solid #aaa;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}"));

        horizontalLayout->addWidget(button_go);

        horizontalSpacer_2 = new QSpacerItem(20, 20, QSizePolicy::Maximum, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        button_del = new QPushButton(widget);
        button_del->setObjectName(QString::fromUtf8("button_del"));
        button_del->setEnabled(false);
        button_del->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #fff, stop: 1 #888);\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"color: #333;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}\n"
"QPushButton:!enabled{\n"
"color: #aaa;\n"
"border: 2px solid #aaa;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #888, stop: 1 #fff);\n"
"}"));

        horizontalLayout->addWidget(button_del);


        verticalLayout_5->addLayout(horizontalLayout);

        verticalSpacer_2 = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer_2);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        current_goal_label = new QLabel(widget);
        current_goal_label->setObjectName(QString::fromUtf8("current_goal_label"));
        sizePolicy4.setHeightForWidth(current_goal_label->sizePolicy().hasHeightForWidth());
        current_goal_label->setSizePolicy(sizePolicy4);
        current_goal_label->setMaximumSize(QSize(240, 20));

        verticalLayout_4->addWidget(current_goal_label);

        current_goal = new QTextEdit(widget);
        current_goal->setObjectName(QString::fromUtf8("current_goal"));
        sizePolicy4.setHeightForWidth(current_goal->sizePolicy().hasHeightForWidth());
        current_goal->setSizePolicy(sizePolicy4);
        current_goal->setMaximumSize(QSize(230, 30));

        verticalLayout_4->addWidget(current_goal);


        verticalLayout_5->addLayout(verticalLayout_4);

        verticalSpacer_4 = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer_4);

        button_estop = new QPushButton(widget);
        button_estop->setObjectName(QString::fromUtf8("button_estop"));
        QSizePolicy sizePolicy5(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(button_estop->sizePolicy().hasHeightForWidth());
        button_estop->setSizePolicy(sizePolicy5);
        button_estop->setMinimumSize(QSize(94, 40));
        button_estop->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"color: black;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 #fff, stop: 1 red);\n"
"min-width: 80px;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"color: red;\n"
"border: 2px solid #555;\n"
"border-radius: 11px;\n"
"padding: 5px;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 black, stop: 1 white);\n"
"}\n"
"\n"
"QPushButton:checked{\n"
"color: red;\n"
"border: 2px solid #aaa;\n"
"background: qradialgradient(cx: 0.3, cy: -0.4,\n"
"fx: 0.3, fy: -0.4,\n"
"radius: 1.35, stop: 0 red, stop: 1 white);\n"
"}"));
        button_estop->setCheckable(true);

        verticalLayout_5->addWidget(button_estop);

        tab_manager->addTab(tab_displays, QString());
        tab_logging = new QWidget();
        tab_logging->setObjectName(QString::fromUtf8("tab_logging"));
        log_groupBox = new QGroupBox(tab_logging);
        log_groupBox->setObjectName(QString::fromUtf8("log_groupBox"));
        log_groupBox->setGeometry(QRect(9, 9, 891, 631));
        QSizePolicy sizePolicy6(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(log_groupBox->sizePolicy().hasHeightForWidth());
        log_groupBox->setSizePolicy(sizePolicy6);
        verticalLayout = new QVBoxLayout(log_groupBox);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        view_logging = new QListView(log_groupBox);
        view_logging->setObjectName(QString::fromUtf8("view_logging"));

        verticalLayout->addWidget(view_logging);

        cmdvel_frame = new QFrame(log_groupBox);
        cmdvel_frame->setObjectName(QString::fromUtf8("cmdvel_frame"));
        cmdvel_frame->setFrameShape(QFrame::StyledPanel);
        cmdvel_frame->setFrameShadow(QFrame::Raised);
        horizontalLayout_2 = new QHBoxLayout(cmdvel_frame);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        robotSpeedLabel = new QLabel(cmdvel_frame);
        robotSpeedLabel->setObjectName(QString::fromUtf8("robotSpeedLabel"));
        sizePolicy4.setHeightForWidth(robotSpeedLabel->sizePolicy().hasHeightForWidth());
        robotSpeedLabel->setSizePolicy(sizePolicy4);
        robotSpeedLabel->setMinimumSize(QSize(211, 31));
        robotSpeedLabel->setMaximumSize(QSize(400, 31));
        QFont font;
        font.setPointSize(10);
        robotSpeedLabel->setFont(font);
        robotSpeedLabel->setStyleSheet(QString::fromUtf8("color: rgb(255,255,255);\n"
"background:rgba(0, 205, 0,190)\n"
""));
        robotSpeedLabel->setFrameShape(QFrame::Box);
        robotSpeedLabel->setFrameShadow(QFrame::Plain);
        robotSpeedLabel->setLineWidth(2);
        robotSpeedLabel->setAlignment(Qt::AlignCenter);

        horizontalLayout_2->addWidget(robotSpeedLabel);

        robotLinearLabel = new QLabel(cmdvel_frame);
        robotLinearLabel->setObjectName(QString::fromUtf8("robotLinearLabel"));
        sizePolicy4.setHeightForWidth(robotLinearLabel->sizePolicy().hasHeightForWidth());
        robotLinearLabel->setSizePolicy(sizePolicy4);
        robotLinearLabel->setMinimumSize(QSize(111, 34));
        robotLinearLabel->setMaximumSize(QSize(126, 34));
        robotLinearLabel->setFont(font);
        robotLinearLabel->setStyleSheet(QString::fromUtf8("background:qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(230, 230, 230, 255), stop:1 rgba(255, 255, 255, 255))"));
        robotLinearLabel->setFrameShape(QFrame::Box);

        horizontalLayout_2->addWidget(robotLinearLabel);

        robotLinear = new QLCDNumber(cmdvel_frame);
        robotLinear->setObjectName(QString::fromUtf8("robotLinear"));
        sizePolicy4.setHeightForWidth(robotLinear->sizePolicy().hasHeightForWidth());
        robotLinear->setSizePolicy(sizePolicy4);
        robotLinear->setMinimumSize(QSize(60, 34));
        robotLinear->setBaseSize(QSize(0, 0));
        robotLinear->setFont(font);
        robotLinear->setStyleSheet(QString::fromUtf8("color: red;  background: rgba(255,255,255,200)"));
        robotLinear->setSmallDecimalPoint(true);
        robotLinear->setNumDigits(4);
        robotLinear->setDigitCount(4);
        robotLinear->setSegmentStyle(QLCDNumber::Flat);
        robotLinear->setProperty("intValue", QVariant(0));

        horizontalLayout_2->addWidget(robotLinear);

        robotAngularLabel = new QLabel(cmdvel_frame);
        robotAngularLabel->setObjectName(QString::fromUtf8("robotAngularLabel"));
        sizePolicy4.setHeightForWidth(robotAngularLabel->sizePolicy().hasHeightForWidth());
        robotAngularLabel->setSizePolicy(sizePolicy4);
        robotAngularLabel->setMinimumSize(QSize(126, 34));
        robotAngularLabel->setMaximumSize(QSize(126, 34));
        QFont font1;
        font1.setPointSize(10);
        font1.setItalic(false);
        robotAngularLabel->setFont(font1);
        robotAngularLabel->setStyleSheet(QString::fromUtf8("background:qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(230, 230, 230, 255), stop:1 rgba(255, 255, 255, 255))"));
        robotAngularLabel->setFrameShape(QFrame::Box);

        horizontalLayout_2->addWidget(robotAngularLabel);

        robotAngular = new QLCDNumber(cmdvel_frame);
        robotAngular->setObjectName(QString::fromUtf8("robotAngular"));
        sizePolicy4.setHeightForWidth(robotAngular->sizePolicy().hasHeightForWidth());
        robotAngular->setSizePolicy(sizePolicy4);
        robotAngular->setMinimumSize(QSize(60, 34));
        robotAngular->setBaseSize(QSize(0, 0));
        robotAngular->setFont(font);
        robotAngular->setStyleSheet(QString::fromUtf8("color: red;  background: rgba(255,255,255,200)"));
        robotAngular->setSmallDecimalPoint(true);
        robotAngular->setNumDigits(4);
        robotAngular->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_2->addWidget(robotAngular);


        verticalLayout->addWidget(cmdvel_frame);

        pose_frame = new QFrame(log_groupBox);
        pose_frame->setObjectName(QString::fromUtf8("pose_frame"));
        pose_frame->setStyleSheet(QString::fromUtf8("background: rgba()"));
        pose_frame->setLocale(QLocale(QLocale::C, QLocale::AnyCountry));
        pose_frame->setFrameShape(QFrame::StyledPanel);
        pose_frame->setFrameShadow(QFrame::Raised);
        horizontalLayout_3 = new QHBoxLayout(pose_frame);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        robotPoseLabel = new QLabel(pose_frame);
        robotPoseLabel->setObjectName(QString::fromUtf8("robotPoseLabel"));
        sizePolicy4.setHeightForWidth(robotPoseLabel->sizePolicy().hasHeightForWidth());
        robotPoseLabel->setSizePolicy(sizePolicy4);
        robotPoseLabel->setMinimumSize(QSize(211, 31));
        robotPoseLabel->setMaximumSize(QSize(400, 31));
        robotPoseLabel->setFont(font);
        robotPoseLabel->setStyleSheet(QString::fromUtf8("color: rgb(255,255,255);\n"
"background:rgba(0, 205, 0,190)\n"
""));
        robotPoseLabel->setFrameShape(QFrame::Box);
        robotPoseLabel->setFrameShadow(QFrame::Plain);
        robotPoseLabel->setLineWidth(2);
        robotPoseLabel->setAlignment(Qt::AlignCenter);

        horizontalLayout_3->addWidget(robotPoseLabel);

        robotXLabel = new QLabel(pose_frame);
        robotXLabel->setObjectName(QString::fromUtf8("robotXLabel"));
        sizePolicy4.setHeightForWidth(robotXLabel->sizePolicy().hasHeightForWidth());
        robotXLabel->setSizePolicy(sizePolicy4);
        robotXLabel->setMinimumSize(QSize(54, 34));
        robotXLabel->setMaximumSize(QSize(54, 34));
        robotXLabel->setFont(font);
        robotXLabel->setStyleSheet(QString::fromUtf8("background:qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(230, 230, 230, 255), stop:1 rgba(255, 255, 255, 255))"));
        robotXLabel->setFrameShape(QFrame::Box);

        horizontalLayout_3->addWidget(robotXLabel);

        robotX = new QLCDNumber(pose_frame);
        robotX->setObjectName(QString::fromUtf8("robotX"));
        sizePolicy4.setHeightForWidth(robotX->sizePolicy().hasHeightForWidth());
        robotX->setSizePolicy(sizePolicy4);
        robotX->setMinimumSize(QSize(131, 40));
        robotX->setBaseSize(QSize(0, 0));
        robotX->setFont(font);
        robotX->setStyleSheet(QString::fromUtf8("color: red;  background: rgba(255,255,255,200)"));
        robotX->setSmallDecimalPoint(true);
        robotX->setNumDigits(4);
        robotX->setDigitCount(4);
        robotX->setSegmentStyle(QLCDNumber::Flat);
        robotX->setProperty("intValue", QVariant(0));

        horizontalLayout_3->addWidget(robotX);

        robotYLabel = new QLabel(pose_frame);
        robotYLabel->setObjectName(QString::fromUtf8("robotYLabel"));
        sizePolicy4.setHeightForWidth(robotYLabel->sizePolicy().hasHeightForWidth());
        robotYLabel->setSizePolicy(sizePolicy4);
        robotYLabel->setMinimumSize(QSize(54, 34));
        robotYLabel->setMaximumSize(QSize(54, 34));
        robotYLabel->setFont(font1);
        robotYLabel->setStyleSheet(QString::fromUtf8("background:qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(230, 230, 230, 255), stop:1 rgba(255, 255, 255, 255))"));
        robotYLabel->setFrameShape(QFrame::Box);

        horizontalLayout_3->addWidget(robotYLabel);

        robotY = new QLCDNumber(pose_frame);
        robotY->setObjectName(QString::fromUtf8("robotY"));
        sizePolicy4.setHeightForWidth(robotY->sizePolicy().hasHeightForWidth());
        robotY->setSizePolicy(sizePolicy4);
        robotY->setMinimumSize(QSize(131, 40));
        robotY->setBaseSize(QSize(0, 0));
        robotY->setFont(font);
        robotY->setStyleSheet(QString::fromUtf8("color: red;  background: rgba(255,255,255,200)"));
        robotY->setSmallDecimalPoint(true);
        robotY->setNumDigits(4);
        robotY->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_3->addWidget(robotY);


        verticalLayout->addWidget(pose_frame);

        tab_manager->addTab(tab_logging, QString());

        hboxLayout->addWidget(tab_manager);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 941, 23));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        menu_Help = new QMenu(menubar);
        menu_Help->setObjectName(QString::fromUtf8("menu_Help"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menubar->addAction(menu_Help->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);
        menu_Help->addAction(actionAbout);
        menu_Help->addAction(actionAbout_Qt);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);
        location_combo->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        map_label->setToolTip(QApplication::translate("MainWindowDesign", "Use Ctrl+mouse scroll to zoom", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        map_label->setText(QApplication::translate("MainWindowDesign", "Map display area", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        button_connect->setToolTip(QApplication::translate("MainWindowDesign", "Connect to ROS Master and load map", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        button_connect->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        button_connect->setText(QApplication::translate("MainWindowDesign", "&Connect", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        setInitPose->setText(QApplication::translate("MainWindowDesign", "&Reposition Robot", 0, QApplication::UnicodeUTF8));
        location_combo->clear();
        location_combo->insertItems(0, QStringList()
         << QApplication::translate("MainWindowDesign", "Home", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindowDesign", "Lift", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindowDesign", "Pillar", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindowDesign", "TwoTwo", 0, QApplication::UnicodeUTF8)
        );
        button_add->setText(QApplication::translate("MainWindowDesign", "&Add goal", 0, QApplication::UnicodeUTF8));
        goals_label->setText(QApplication::translate("MainWindowDesign", "Target locations:", 0, QApplication::UnicodeUTF8));
        button_go->setText(QApplication::translate("MainWindowDesign", "&Go", 0, QApplication::UnicodeUTF8));
        button_del->setText(QApplication::translate("MainWindowDesign", "&Delete goal", 0, QApplication::UnicodeUTF8));
        current_goal_label->setText(QApplication::translate("MainWindowDesign", "Now heading towards", 0, QApplication::UnicodeUTF8));
        current_goal->setHtml(QApplication::translate("MainWindowDesign", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"></p></body></html>", 0, QApplication::UnicodeUTF8));
        button_estop->setText(QApplication::translate("MainWindowDesign", "&Emergency Stop", 0, QApplication::UnicodeUTF8));
        tab_manager->setTabText(tab_manager->indexOf(tab_displays), QApplication::translate("MainWindowDesign", "Display", 0, QApplication::UnicodeUTF8));
        log_groupBox->setTitle(QApplication::translate("MainWindowDesign", "Logging", 0, QApplication::UnicodeUTF8));
        robotSpeedLabel->setText(QApplication::translate("MainWindowDesign", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Lucida Grande'; font-size:12pt; font-weight:600; color:#ffffff;\">Command Speed</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        robotLinearLabel->setText(QApplication::translate("MainWindowDesign", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Linear [m/s]:</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        robotAngularLabel->setText(QApplication::translate("MainWindowDesign", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Sans'; font-weight:600;\">Angular [rad/s]:</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        robotPoseLabel->setText(QApplication::translate("MainWindowDesign", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Lucida Grande'; font-size:12pt; font-weight:600; color:#ffffff;\">Current Position</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        robotXLabel->setText(QApplication::translate("MainWindowDesign", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">x [m]:</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        robotYLabel->setText(QApplication::translate("MainWindowDesign", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">y [m]:</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        tab_manager->setTabText(tab_manager->indexOf(tab_logging), QApplication::translate("MainWindowDesign", "Logs", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&File", 0, QApplication::UnicodeUTF8));
        menu_Help->setTitle(QApplication::translate("MainWindowDesign", "&Help", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
