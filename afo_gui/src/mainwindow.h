
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string.h>
#include <iostream>

#include "qcustomplot.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

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
    void togglePlot();
    void updateLog(QString s);
    void set_emergency(bool on);
    
    void updatePlot(int dataType);
    void initPlot();

public Q_SLOTS:
    void buttonClicked();
    void plotSoleLeft();
    void plotSoleRight();
    void plotPlantar();
    void plotDorsi();


private:
    QNode qnode;
    Ui::MainWindow *ui;
    int logNum = 0;
    int max_log = 20;
    int voltPlotMaxNum = 400;
    int motorPlotMaxNum = 400;
    bool is_plot = false;

    QVector<double> t_v_l, t_v_r, t_m_p, t_m_d;
    QVector<double> v_l[6];
    QVector<double> v_r[6];
    QVector<double> m_p[2];
    QVector<double> m_d[4];

};


void appendCropQVector(QVector<double> *vector, double data, int maxNum);
#endif // MAINWINDOW_H
