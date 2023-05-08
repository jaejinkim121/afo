
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string.h>
#include <iostream>

//#include "ros/ros.h"
//#include "std_msgs/Float32MultiArray.h"
//#include "std_msgs/Int16MultiArray.h"
//#include "std_msgs/Float32.h"
//#include "std_msgs/Int16.h"
//#include "std_msgs/Bool.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#define SOLE_LEFT 1
#define SOLE_RIGHT 2
#define MOTOR_PLANTAR 3
#define MOTOR_DORSI 4

class MainWindow : public QMainWindow

{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void toggleVoltPlot();
    void updateLog(QString s);
    void set_emergency(bool on);
    void callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackSoleRight(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackPlantar(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackDorsi(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void updatePlot(int dataType);
    void initPlot();

public slots:
    void buttonClicked();

private:
    Ui::MainWindow *ui;
    int logNum = 0;
    int max_log = 20;
    int voltPlotMaxNum = 200;
    int motorPlotMaxNum = 400;
    bool is;
    bool is_plot = false;

    ros::Publisher afo_gui_sole_calibration_pub;
    ros::Publisher afo_gui_max_torque_pub;
    ros::Subscriber afo_soleSensor_left_sub;
    ros::Subscriber afo_soleSensor_right_sub;
    ros::Subscriber afo_plantar_command_sub;
    ros::Subscriber afo_dorsi_command_sub;

    ros::NodeHandle nh;

    double t_begin;


    QVector<double> t_v_l, t_v_r, t_m_p, t_m_d;
    QVector<double> v_l[6];
    QVector<double> v_r[6];
    QVector<double> m_p[2];
    QVector<double> m_d[4];

};


void appendCropQVector(QVector<double> *vector, double data, int maxNum);
#endif // MAINWINDOW_H
