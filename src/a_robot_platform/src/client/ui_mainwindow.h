/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFormLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget;
    QWidget *tab_3;
    QGroupBox *gBx_imu;
    QWidget *form_3;
    QFormLayout *formLayout_3;
    QLabel *lbl_9;
    QLabel *lbl_10;
    QLabel *lbl_11;
    QHBoxLayout *horizontalLayout_7;
    QLabel *lbl_acc;
    QLabel *lbl_gyr;
    QHBoxLayout *horizontalLayout_8;
    QLabel *lbl_acc_x;
    QLabel *lbl_gyr_x;
    QHBoxLayout *horizontalLayout_9;
    QLabel *lbl_acc_y;
    QLabel *lbl_gyr_y;
    QHBoxLayout *horizontalLayout_10;
    QLabel *lbl_acc_z;
    QLabel *lbl_gyr_z;
    QGroupBox *gBx_control;
    QPushButton *pBtn_key_control_open;
    QWidget *form;
    QFormLayout *formLayout;
    QLabel *lbl_1;
    QLabel *lbl_2;
    QLabel *lbl_3;
    QHBoxLayout *horizontalLayout_3;
    QLabel *lbl_velocity;
    QLabel *lbl_omega;
    QHBoxLayout *horizontalLayout_4;
    QLabel *lbl_vel_max;
    QLabel *lbl_ome_max;
    QHBoxLayout *horizontalLayout_5;
    QLabel *lbl_vel_exp;
    QLabel *lbl_ome_exp;
    QHBoxLayout *horizontalLayout_6;
    QLabel *lbl_vel_ret;
    QLabel *lbl_ome_ret;
    QGroupBox *gBx_Pose;
    QWidget *form_5;
    QFormLayout *formLayout_5;
    QLabel *lbl_15;
    QLabel *lbl_pose_r;
    QLabel *lbl_16;
    QLabel *lbl_pose_p;
    QLabel *lbl_17;
    QLabel *lbl_pose_h;
    QLabel *lbl_euler;
    QWidget *form_6;
    QFormLayout *formLayout_7;
    QLabel *lbl_21;
    QLabel *lbl_pose_x;
    QLabel *lbl_22;
    QLabel *lbl_pose_y;
    QLabel *lbl_23;
    QLabel *lbl_pose_z;
    QLabel *lbl_pose;
    QGroupBox *groupBox;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *pBtn_start2connect;
    QLabel *lbl_7;
    QLineEdit *lEdit_ip;
    QLabel *lbl_8;
    QLineEdit *lEdit_port;
    QGroupBox *groupBox_2;
    QGroupBox *groupBox_3;
    QWidget *tab_4;
    QLabel *label_main;
    QGroupBox *groupBox_4;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout_2;
    QRadioButton *rb_all;
    QRadioButton *rb_2m;
    QRadioButton *rb_1m;
    QWidget *tab_5;
    QLabel *label_Ultra;
    QGroupBox *groupBox_5;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_3;
    QCheckBox *ultra1;
    QCheckBox *ultra2;
    QCheckBox *ultra3;
    QCheckBox *ultra4;
    QCheckBox *ultra5;
    QCheckBox *ultra6;
    QCheckBox *ultra7;
    QCheckBox *ultra8;
    QCheckBox *ultraAll;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_4;
    QLabel *lb_ultra1;
    QLabel *lb_ultra2;
    QLabel *lb_ultra3;
    QLabel *lb_ultra4;
    QLabel *lb_ultra5;
    QLabel *lb_ultra6;
    QLabel *lb_ultra7;
    QLabel *lb_ultra8;
    QGroupBox *groupBox_6;
    QCheckBox *Ultra_Area;
    QCheckBox *Obsta_OutLine;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1024, 971);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMaximumSize(QSize(16777209, 16777215));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(20, 0, 1011, 901));
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        gBx_imu = new QGroupBox(tab_3);
        gBx_imu->setObjectName(QString::fromUtf8("gBx_imu"));
        gBx_imu->setGeometry(QRect(350, 250, 311, 161));
        form_3 = new QWidget(gBx_imu);
        form_3->setObjectName(QString::fromUtf8("form_3"));
        form_3->setGeometry(QRect(11, 31, 291, 111));
        formLayout_3 = new QFormLayout(form_3);
        formLayout_3->setSpacing(6);
        formLayout_3->setContentsMargins(11, 11, 11, 11);
        formLayout_3->setObjectName(QString::fromUtf8("formLayout_3"));
        formLayout_3->setContentsMargins(0, 0, 0, 0);
        lbl_9 = new QLabel(form_3);
        lbl_9->setObjectName(QString::fromUtf8("lbl_9"));

        formLayout_3->setWidget(2, QFormLayout::LabelRole, lbl_9);

        lbl_10 = new QLabel(form_3);
        lbl_10->setObjectName(QString::fromUtf8("lbl_10"));

        formLayout_3->setWidget(3, QFormLayout::LabelRole, lbl_10);

        lbl_11 = new QLabel(form_3);
        lbl_11->setObjectName(QString::fromUtf8("lbl_11"));

        formLayout_3->setWidget(4, QFormLayout::LabelRole, lbl_11);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        lbl_acc = new QLabel(form_3);
        lbl_acc->setObjectName(QString::fromUtf8("lbl_acc"));
        lbl_acc->setLayoutDirection(Qt::RightToLeft);
        lbl_acc->setAlignment(Qt::AlignCenter);

        horizontalLayout_7->addWidget(lbl_acc);

        lbl_gyr = new QLabel(form_3);
        lbl_gyr->setObjectName(QString::fromUtf8("lbl_gyr"));
        lbl_gyr->setStyleSheet(QString::fromUtf8(""));
        lbl_gyr->setAlignment(Qt::AlignCenter);

        horizontalLayout_7->addWidget(lbl_gyr);


        formLayout_3->setLayout(1, QFormLayout::FieldRole, horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        lbl_acc_x = new QLabel(form_3);
        lbl_acc_x->setObjectName(QString::fromUtf8("lbl_acc_x"));
        lbl_acc_x->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_acc_x->setAlignment(Qt::AlignCenter);

        horizontalLayout_8->addWidget(lbl_acc_x);

        lbl_gyr_x = new QLabel(form_3);
        lbl_gyr_x->setObjectName(QString::fromUtf8("lbl_gyr_x"));
        lbl_gyr_x->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_gyr_x->setAlignment(Qt::AlignCenter);

        horizontalLayout_8->addWidget(lbl_gyr_x);


        formLayout_3->setLayout(2, QFormLayout::FieldRole, horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        lbl_acc_y = new QLabel(form_3);
        lbl_acc_y->setObjectName(QString::fromUtf8("lbl_acc_y"));
        lbl_acc_y->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_acc_y->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(lbl_acc_y);

        lbl_gyr_y = new QLabel(form_3);
        lbl_gyr_y->setObjectName(QString::fromUtf8("lbl_gyr_y"));
        lbl_gyr_y->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_gyr_y->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(lbl_gyr_y);


        formLayout_3->setLayout(3, QFormLayout::FieldRole, horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        lbl_acc_z = new QLabel(form_3);
        lbl_acc_z->setObjectName(QString::fromUtf8("lbl_acc_z"));
        lbl_acc_z->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_acc_z->setAlignment(Qt::AlignCenter);

        horizontalLayout_10->addWidget(lbl_acc_z);

        lbl_gyr_z = new QLabel(form_3);
        lbl_gyr_z->setObjectName(QString::fromUtf8("lbl_gyr_z"));
        lbl_gyr_z->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_gyr_z->setAlignment(Qt::AlignCenter);

        horizontalLayout_10->addWidget(lbl_gyr_z);


        formLayout_3->setLayout(4, QFormLayout::FieldRole, horizontalLayout_10);

        gBx_control = new QGroupBox(tab_3);
        gBx_control->setObjectName(QString::fromUtf8("gBx_control"));
        gBx_control->setGeometry(QRect(670, 20, 311, 221));
        pBtn_key_control_open = new QPushButton(gBx_control);
        pBtn_key_control_open->setObjectName(QString::fromUtf8("pBtn_key_control_open"));
        pBtn_key_control_open->setGeometry(QRect(20, 30, 96, 27));
        pBtn_key_control_open->setCheckable(true);
        form = new QWidget(gBx_control);
        form->setObjectName(QString::fromUtf8("form"));
        form->setGeometry(QRect(20, 70, 281, 111));
        formLayout = new QFormLayout(form);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        lbl_1 = new QLabel(form);
        lbl_1->setObjectName(QString::fromUtf8("lbl_1"));

        formLayout->setWidget(2, QFormLayout::LabelRole, lbl_1);

        lbl_2 = new QLabel(form);
        lbl_2->setObjectName(QString::fromUtf8("lbl_2"));

        formLayout->setWidget(3, QFormLayout::LabelRole, lbl_2);

        lbl_3 = new QLabel(form);
        lbl_3->setObjectName(QString::fromUtf8("lbl_3"));

        formLayout->setWidget(4, QFormLayout::LabelRole, lbl_3);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        lbl_velocity = new QLabel(form);
        lbl_velocity->setObjectName(QString::fromUtf8("lbl_velocity"));
        lbl_velocity->setStyleSheet(QString::fromUtf8(""));
        lbl_velocity->setAlignment(Qt::AlignCenter);

        horizontalLayout_3->addWidget(lbl_velocity);

        lbl_omega = new QLabel(form);
        lbl_omega->setObjectName(QString::fromUtf8("lbl_omega"));
        lbl_omega->setLayoutDirection(Qt::RightToLeft);
        lbl_omega->setAlignment(Qt::AlignCenter);

        horizontalLayout_3->addWidget(lbl_omega);


        formLayout->setLayout(1, QFormLayout::FieldRole, horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        lbl_vel_max = new QLabel(form);
        lbl_vel_max->setObjectName(QString::fromUtf8("lbl_vel_max"));
        lbl_vel_max->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_vel_max->setAlignment(Qt::AlignCenter);

        horizontalLayout_4->addWidget(lbl_vel_max);

        lbl_ome_max = new QLabel(form);
        lbl_ome_max->setObjectName(QString::fromUtf8("lbl_ome_max"));
        lbl_ome_max->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_ome_max->setAlignment(Qt::AlignCenter);

        horizontalLayout_4->addWidget(lbl_ome_max);


        formLayout->setLayout(2, QFormLayout::FieldRole, horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        lbl_vel_exp = new QLabel(form);
        lbl_vel_exp->setObjectName(QString::fromUtf8("lbl_vel_exp"));
        lbl_vel_exp->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_vel_exp->setAlignment(Qt::AlignCenter);

        horizontalLayout_5->addWidget(lbl_vel_exp);

        lbl_ome_exp = new QLabel(form);
        lbl_ome_exp->setObjectName(QString::fromUtf8("lbl_ome_exp"));
        lbl_ome_exp->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_ome_exp->setAlignment(Qt::AlignCenter);

        horizontalLayout_5->addWidget(lbl_ome_exp);


        formLayout->setLayout(3, QFormLayout::FieldRole, horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        lbl_vel_ret = new QLabel(form);
        lbl_vel_ret->setObjectName(QString::fromUtf8("lbl_vel_ret"));
        lbl_vel_ret->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_vel_ret->setAlignment(Qt::AlignCenter);

        horizontalLayout_6->addWidget(lbl_vel_ret);

        lbl_ome_ret = new QLabel(form);
        lbl_ome_ret->setObjectName(QString::fromUtf8("lbl_ome_ret"));
        lbl_ome_ret->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_ome_ret->setAlignment(Qt::AlignCenter);

        horizontalLayout_6->addWidget(lbl_ome_ret);


        formLayout->setLayout(4, QFormLayout::FieldRole, horizontalLayout_6);

        gBx_Pose = new QGroupBox(tab_3);
        gBx_Pose->setObjectName(QString::fromUtf8("gBx_Pose"));
        gBx_Pose->setGeometry(QRect(670, 250, 311, 161));
        form_5 = new QWidget(gBx_Pose);
        form_5->setObjectName(QString::fromUtf8("form_5"));
        form_5->setGeometry(QRect(200, 30, 101, 111));
        formLayout_5 = new QFormLayout(form_5);
        formLayout_5->setSpacing(6);
        formLayout_5->setContentsMargins(11, 11, 11, 11);
        formLayout_5->setObjectName(QString::fromUtf8("formLayout_5"));
        formLayout_5->setContentsMargins(0, 0, 0, 0);
        lbl_15 = new QLabel(form_5);
        lbl_15->setObjectName(QString::fromUtf8("lbl_15"));

        formLayout_5->setWidget(2, QFormLayout::LabelRole, lbl_15);

        lbl_pose_r = new QLabel(form_5);
        lbl_pose_r->setObjectName(QString::fromUtf8("lbl_pose_r"));
        lbl_pose_r->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_pose_r->setAlignment(Qt::AlignCenter);

        formLayout_5->setWidget(2, QFormLayout::FieldRole, lbl_pose_r);

        lbl_16 = new QLabel(form_5);
        lbl_16->setObjectName(QString::fromUtf8("lbl_16"));

        formLayout_5->setWidget(3, QFormLayout::LabelRole, lbl_16);

        lbl_pose_p = new QLabel(form_5);
        lbl_pose_p->setObjectName(QString::fromUtf8("lbl_pose_p"));
        lbl_pose_p->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_pose_p->setAlignment(Qt::AlignCenter);

        formLayout_5->setWidget(3, QFormLayout::FieldRole, lbl_pose_p);

        lbl_17 = new QLabel(form_5);
        lbl_17->setObjectName(QString::fromUtf8("lbl_17"));

        formLayout_5->setWidget(4, QFormLayout::LabelRole, lbl_17);

        lbl_pose_h = new QLabel(form_5);
        lbl_pose_h->setObjectName(QString::fromUtf8("lbl_pose_h"));
        lbl_pose_h->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_pose_h->setAlignment(Qt::AlignCenter);

        formLayout_5->setWidget(4, QFormLayout::FieldRole, lbl_pose_h);

        lbl_euler = new QLabel(form_5);
        lbl_euler->setObjectName(QString::fromUtf8("lbl_euler"));
        lbl_euler->setLayoutDirection(Qt::RightToLeft);
        lbl_euler->setAlignment(Qt::AlignCenter);

        formLayout_5->setWidget(1, QFormLayout::FieldRole, lbl_euler);

        form_6 = new QWidget(gBx_Pose);
        form_6->setObjectName(QString::fromUtf8("form_6"));
        form_6->setGeometry(QRect(10, 30, 101, 111));
        formLayout_7 = new QFormLayout(form_6);
        formLayout_7->setSpacing(6);
        formLayout_7->setContentsMargins(11, 11, 11, 11);
        formLayout_7->setObjectName(QString::fromUtf8("formLayout_7"));
        formLayout_7->setContentsMargins(0, 0, 0, 0);
        lbl_21 = new QLabel(form_6);
        lbl_21->setObjectName(QString::fromUtf8("lbl_21"));

        formLayout_7->setWidget(2, QFormLayout::LabelRole, lbl_21);

        lbl_pose_x = new QLabel(form_6);
        lbl_pose_x->setObjectName(QString::fromUtf8("lbl_pose_x"));
        lbl_pose_x->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_pose_x->setAlignment(Qt::AlignCenter);

        formLayout_7->setWidget(2, QFormLayout::FieldRole, lbl_pose_x);

        lbl_22 = new QLabel(form_6);
        lbl_22->setObjectName(QString::fromUtf8("lbl_22"));

        formLayout_7->setWidget(3, QFormLayout::LabelRole, lbl_22);

        lbl_pose_y = new QLabel(form_6);
        lbl_pose_y->setObjectName(QString::fromUtf8("lbl_pose_y"));
        lbl_pose_y->setStyleSheet(QString::fromUtf8("rgb:(255, 85, 0)"));
        lbl_pose_y->setAlignment(Qt::AlignCenter);

        formLayout_7->setWidget(3, QFormLayout::FieldRole, lbl_pose_y);

        lbl_23 = new QLabel(form_6);
        lbl_23->setObjectName(QString::fromUtf8("lbl_23"));

        formLayout_7->setWidget(4, QFormLayout::LabelRole, lbl_23);

        lbl_pose_z = new QLabel(form_6);
        lbl_pose_z->setObjectName(QString::fromUtf8("lbl_pose_z"));
        lbl_pose_z->setStyleSheet(QString::fromUtf8("rgb:(255, 0, 0);"));
        lbl_pose_z->setAlignment(Qt::AlignCenter);

        formLayout_7->setWidget(4, QFormLayout::FieldRole, lbl_pose_z);

        lbl_pose = new QLabel(form_6);
        lbl_pose->setObjectName(QString::fromUtf8("lbl_pose"));
        lbl_pose->setLayoutDirection(Qt::RightToLeft);
        lbl_pose->setAlignment(Qt::AlignCenter);

        formLayout_7->setWidget(1, QFormLayout::FieldRole, lbl_pose);

        groupBox = new QGroupBox(tab_3);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(350, 20, 311, 221));
        verticalLayoutWidget = new QWidget(groupBox);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 30, 291, 181));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        pBtn_start2connect = new QPushButton(verticalLayoutWidget);
        pBtn_start2connect->setObjectName(QString::fromUtf8("pBtn_start2connect"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pBtn_start2connect->sizePolicy().hasHeightForWidth());
        pBtn_start2connect->setSizePolicy(sizePolicy1);
        pBtn_start2connect->setCheckable(true);

        verticalLayout->addWidget(pBtn_start2connect);

        lbl_7 = new QLabel(verticalLayoutWidget);
        lbl_7->setObjectName(QString::fromUtf8("lbl_7"));
        lbl_7->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(lbl_7);

        lEdit_ip = new QLineEdit(verticalLayoutWidget);
        lEdit_ip->setObjectName(QString::fromUtf8("lEdit_ip"));
        lEdit_ip->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(lEdit_ip);

        lbl_8 = new QLabel(verticalLayoutWidget);
        lbl_8->setObjectName(QString::fromUtf8("lbl_8"));
        lbl_8->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(lbl_8);

        lEdit_port = new QLineEdit(verticalLayoutWidget);
        lEdit_port->setObjectName(QString::fromUtf8("lEdit_port"));
        lEdit_port->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(lEdit_port);

        groupBox_2 = new QGroupBox(tab_3);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 20, 311, 391));
        groupBox_3 = new QGroupBox(tab_3);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(20, 420, 961, 441));
        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        label_main = new QLabel(tab_4);
        label_main->setObjectName(QString::fromUtf8("label_main"));
        label_main->setGeometry(QRect(10, 4, 821, 841));
        groupBox_4 = new QGroupBox(tab_4);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(849, 19, 151, 281));
        layoutWidget = new QWidget(groupBox_4);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(14, 40, 121, 231));
        verticalLayout_2 = new QVBoxLayout(layoutWidget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        rb_all = new QRadioButton(layoutWidget);
        rb_all->setObjectName(QString::fromUtf8("rb_all"));

        verticalLayout_2->addWidget(rb_all);

        rb_2m = new QRadioButton(layoutWidget);
        rb_2m->setObjectName(QString::fromUtf8("rb_2m"));

        verticalLayout_2->addWidget(rb_2m);

        rb_1m = new QRadioButton(layoutWidget);
        rb_1m->setObjectName(QString::fromUtf8("rb_1m"));

        verticalLayout_2->addWidget(rb_1m);

        tabWidget->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        label_Ultra = new QLabel(tab_5);
        label_Ultra->setObjectName(QString::fromUtf8("label_Ultra"));
        label_Ultra->setGeometry(QRect(20, 24, 751, 831));
        groupBox_5 = new QGroupBox(tab_5);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(789, 19, 211, 291));
        layoutWidget1 = new QWidget(groupBox_5);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 30, 91, 239));
        verticalLayout_3 = new QVBoxLayout(layoutWidget1);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        ultra1 = new QCheckBox(layoutWidget1);
        ultra1->setObjectName(QString::fromUtf8("ultra1"));

        verticalLayout_3->addWidget(ultra1);

        ultra2 = new QCheckBox(layoutWidget1);
        ultra2->setObjectName(QString::fromUtf8("ultra2"));

        verticalLayout_3->addWidget(ultra2);

        ultra3 = new QCheckBox(layoutWidget1);
        ultra3->setObjectName(QString::fromUtf8("ultra3"));

        verticalLayout_3->addWidget(ultra3);

        ultra4 = new QCheckBox(layoutWidget1);
        ultra4->setObjectName(QString::fromUtf8("ultra4"));

        verticalLayout_3->addWidget(ultra4);

        ultra5 = new QCheckBox(layoutWidget1);
        ultra5->setObjectName(QString::fromUtf8("ultra5"));

        verticalLayout_3->addWidget(ultra5);

        ultra6 = new QCheckBox(layoutWidget1);
        ultra6->setObjectName(QString::fromUtf8("ultra6"));

        verticalLayout_3->addWidget(ultra6);

        ultra7 = new QCheckBox(layoutWidget1);
        ultra7->setObjectName(QString::fromUtf8("ultra7"));

        verticalLayout_3->addWidget(ultra7);

        ultra8 = new QCheckBox(layoutWidget1);
        ultra8->setObjectName(QString::fromUtf8("ultra8"));

        verticalLayout_3->addWidget(ultra8);

        ultraAll = new QCheckBox(layoutWidget1);
        ultraAll->setObjectName(QString::fromUtf8("ultraAll"));

        verticalLayout_3->addWidget(ultraAll);

        layoutWidget2 = new QWidget(groupBox_5);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(111, 30, 81, 211));
        verticalLayout_4 = new QVBoxLayout(layoutWidget2);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        lb_ultra1 = new QLabel(layoutWidget2);
        lb_ultra1->setObjectName(QString::fromUtf8("lb_ultra1"));

        verticalLayout_4->addWidget(lb_ultra1);

        lb_ultra2 = new QLabel(layoutWidget2);
        lb_ultra2->setObjectName(QString::fromUtf8("lb_ultra2"));

        verticalLayout_4->addWidget(lb_ultra2);

        lb_ultra3 = new QLabel(layoutWidget2);
        lb_ultra3->setObjectName(QString::fromUtf8("lb_ultra3"));

        verticalLayout_4->addWidget(lb_ultra3);

        lb_ultra4 = new QLabel(layoutWidget2);
        lb_ultra4->setObjectName(QString::fromUtf8("lb_ultra4"));

        verticalLayout_4->addWidget(lb_ultra4);

        lb_ultra5 = new QLabel(layoutWidget2);
        lb_ultra5->setObjectName(QString::fromUtf8("lb_ultra5"));

        verticalLayout_4->addWidget(lb_ultra5);

        lb_ultra6 = new QLabel(layoutWidget2);
        lb_ultra6->setObjectName(QString::fromUtf8("lb_ultra6"));

        verticalLayout_4->addWidget(lb_ultra6);

        lb_ultra7 = new QLabel(layoutWidget2);
        lb_ultra7->setObjectName(QString::fromUtf8("lb_ultra7"));

        verticalLayout_4->addWidget(lb_ultra7);

        lb_ultra8 = new QLabel(layoutWidget2);
        lb_ultra8->setObjectName(QString::fromUtf8("lb_ultra8"));

        verticalLayout_4->addWidget(lb_ultra8);

        groupBox_6 = new QGroupBox(tab_5);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        groupBox_6->setGeometry(QRect(790, 340, 211, 121));
        Ultra_Area = new QCheckBox(groupBox_6);
        Ultra_Area->setObjectName(QString::fromUtf8("Ultra_Area"));
        Ultra_Area->setGeometry(QRect(10, 30, 101, 21));
        Obsta_OutLine = new QCheckBox(groupBox_6);
        Obsta_OutLine->setObjectName(QString::fromUtf8("Obsta_OutLine"));
        Obsta_OutLine->setGeometry(QRect(10, 80, 91, 21));
        tabWidget->addTab(tab_5, QString());
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1024, 20));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        gBx_imu->setTitle(QApplication::translate("MainWindow", "IMU", 0, QApplication::UnicodeUTF8));
        lbl_9->setText(QApplication::translate("MainWindow", "x", 0, QApplication::UnicodeUTF8));
        lbl_10->setText(QApplication::translate("MainWindow", "y", 0, QApplication::UnicodeUTF8));
        lbl_11->setText(QApplication::translate("MainWindow", "z", 0, QApplication::UnicodeUTF8));
        lbl_acc->setText(QApplication::translate("MainWindow", "acc (m/s^2)", 0, QApplication::UnicodeUTF8));
        lbl_gyr->setText(QApplication::translate("MainWindow", "gyr (rad/s)", 0, QApplication::UnicodeUTF8));
        lbl_acc_x->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lbl_gyr_x->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lbl_acc_y->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lbl_gyr_y->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lbl_acc_z->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        lbl_gyr_z->setText(QApplication::translate("MainWindow", "0", 0, QApplication::UnicodeUTF8));
        gBx_control->setTitle(QApplication::translate("MainWindow", "control", 0, QApplication::UnicodeUTF8));
        pBtn_key_control_open->setText(QApplication::translate("MainWindow", "stoped", 0, QApplication::UnicodeUTF8));
        lbl_1->setText(QApplication::translate("MainWindow", "max", 0, QApplication::UnicodeUTF8));
        lbl_2->setText(QApplication::translate("MainWindow", "expect", 0, QApplication::UnicodeUTF8));
        lbl_3->setText(QApplication::translate("MainWindow", "return", 0, QApplication::UnicodeUTF8));
        lbl_velocity->setText(QApplication::translate("MainWindow", "vel (m/s)", 0, QApplication::UnicodeUTF8));
        lbl_omega->setText(QApplication::translate("MainWindow", "ome (rad/s)", 0, QApplication::UnicodeUTF8));
        lbl_vel_max->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_ome_max->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_vel_exp->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_ome_exp->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_vel_ret->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_ome_ret->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        gBx_Pose->setTitle(QApplication::translate("MainWindow", "Pose", 0, QApplication::UnicodeUTF8));
        lbl_15->setText(QApplication::translate("MainWindow", "r", 0, QApplication::UnicodeUTF8));
        lbl_pose_r->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_16->setText(QApplication::translate("MainWindow", "p", 0, QApplication::UnicodeUTF8));
        lbl_pose_p->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_17->setText(QApplication::translate("MainWindow", "y", 0, QApplication::UnicodeUTF8));
        lbl_pose_h->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_euler->setText(QApplication::translate("MainWindow", "eular (rad)", 0, QApplication::UnicodeUTF8));
        lbl_21->setText(QApplication::translate("MainWindow", "x", 0, QApplication::UnicodeUTF8));
        lbl_pose_x->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_22->setText(QApplication::translate("MainWindow", "y", 0, QApplication::UnicodeUTF8));
        lbl_pose_y->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_23->setText(QApplication::translate("MainWindow", "z", 0, QApplication::UnicodeUTF8));
        lbl_pose_z->setText(QApplication::translate("MainWindow", "0.00", 0, QApplication::UnicodeUTF8));
        lbl_pose->setText(QApplication::translate("MainWindow", "pose (m)", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "NetWork", 0, QApplication::UnicodeUTF8));
        pBtn_start2connect->setText(QApplication::translate("MainWindow", "connect", 0, QApplication::UnicodeUTF8));
        lbl_7->setText(QApplication::translate("MainWindow", "IP", 0, QApplication::UnicodeUTF8));
        lEdit_ip->setText(QApplication::translate("MainWindow", "192.168.1.101", 0, QApplication::UnicodeUTF8));
        lbl_8->setText(QApplication::translate("MainWindow", "port", 0, QApplication::UnicodeUTF8));
        lEdit_port->setText(QApplication::translate("MainWindow", "9527", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "\344\270\262\345\217\243\351\205\215\347\275\256", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "\346\277\200\345\205\211\345\216\237\345\247\213\346\225\260\346\215\256", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "\350\256\276\347\275\256", 0, QApplication::UnicodeUTF8));
        label_main->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "\346\230\276\347\244\272\350\256\276\347\275\256", 0, QApplication::UnicodeUTF8));
        rb_all->setText(QApplication::translate("MainWindow", "\345\216\237\345\247\213\345\233\276\345\203\217", 0, QApplication::UnicodeUTF8));
        rb_2m->setText(QApplication::translate("MainWindow", "2m\344\273\245\345\206\205", 0, QApplication::UnicodeUTF8));
        rb_1m->setText(QApplication::translate("MainWindow", "1m\344\273\245\345\206\205", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "\346\277\200\345\205\211\346\216\242\346\265\213\345\233\276", 0, QApplication::UnicodeUTF8));
        label_Ultra->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("MainWindow", "\350\266\205\345\243\260\345\274\200\345\205\263", 0, QApplication::UnicodeUTF8));
        ultra1->setText(QApplication::translate("MainWindow", "1#\350\266\205\345\243\260\346\263\242", 0, QApplication::UnicodeUTF8));
        ultra2->setText(QApplication::translate("MainWindow", "2#\350\266\205\345\243\260\346\263\242", 0, QApplication::UnicodeUTF8));
        ultra3->setText(QApplication::translate("MainWindow", "3#\350\266\205\345\243\260\346\263\242", 0, QApplication::UnicodeUTF8));
        ultra4->setText(QApplication::translate("MainWindow", "4#\350\266\205\345\243\260\346\263\242", 0, QApplication::UnicodeUTF8));
        ultra5->setText(QApplication::translate("MainWindow", "5#\350\266\205\345\243\260\346\263\242", 0, QApplication::UnicodeUTF8));
        ultra6->setText(QApplication::translate("MainWindow", "6#\350\266\205\345\243\260\346\263\242", 0, QApplication::UnicodeUTF8));
        ultra7->setText(QApplication::translate("MainWindow", "7#\350\266\205\345\243\260\346\263\242", 0, QApplication::UnicodeUTF8));
        ultra8->setText(QApplication::translate("MainWindow", "8#\350\266\205\345\243\260\346\263\242", 0, QApplication::UnicodeUTF8));
        ultraAll->setText(QApplication::translate("MainWindow", "\345\205\250\351\200\211", 0, QApplication::UnicodeUTF8));
        lb_ultra1->setText(QApplication::translate("MainWindow", "0000", 0, QApplication::UnicodeUTF8));
        lb_ultra2->setText(QApplication::translate("MainWindow", "0000", 0, QApplication::UnicodeUTF8));
        lb_ultra3->setText(QApplication::translate("MainWindow", "0000", 0, QApplication::UnicodeUTF8));
        lb_ultra4->setText(QApplication::translate("MainWindow", "0000", 0, QApplication::UnicodeUTF8));
        lb_ultra5->setText(QApplication::translate("MainWindow", "0000", 0, QApplication::UnicodeUTF8));
        lb_ultra6->setText(QApplication::translate("MainWindow", "0000", 0, QApplication::UnicodeUTF8));
        lb_ultra7->setText(QApplication::translate("MainWindow", "0000", 0, QApplication::UnicodeUTF8));
        lb_ultra8->setText(QApplication::translate("MainWindow", "0000", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("MainWindow", "\346\230\276\347\244\272\350\256\276\347\275\256", 0, QApplication::UnicodeUTF8));
        Ultra_Area->setText(QApplication::translate("MainWindow", "\350\266\205\345\243\260\350\214\203\345\233\264\346\230\276\347\244\272", 0, QApplication::UnicodeUTF8));
        Obsta_OutLine->setText(QApplication::translate("MainWindow", "\351\232\234\347\242\215\350\275\256\345\273\223\346\230\276\347\244\272", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QApplication::translate("MainWindow", "\350\266\205\345\243\260\346\263\242\346\216\242\346\265\213\345\233\276", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
