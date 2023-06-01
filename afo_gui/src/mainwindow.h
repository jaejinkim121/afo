
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string.h>
#include <iostream>
#include <math.h>

#include "qnode.hpp"
#include "qcustomplot.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "draw.h"

#define SOLE_LEFT 1
#define SOLE_RIGHT 2
#define MOTOR_PLANTAR 3
#define MOTOR_DORSI 4
#define GAIT_PHASE 5

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow{
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    void togglePlotData();
    void togglePlotSole();
    void soleCalibrationLeft();
    void soleCalibrationRight();
    void setMaxTorque();
    void setCycleTime();
    void setLinkLength();
    void togglePlantarRun();
    void toggleDorsiRun();
    void toggleStreaming();
    void typeKey(char c);
    void deleteKey();
    void toggleTrial();
    void emergencyStop();
    void updateMaxTorqueValue();
    void updateCycleTimeValue();
    void targetLinkIdx();
    void imuZero();
    void togglePage();
    
    void updateLog(QString s);
    void set_emergency(bool on);
    
    void updatePlot(int dataType);
    void initPlot();
    void initSolePlot();
    void updateSolePlot(int side,  float* data);
    void updateRGB(float f);

public Q_SLOTS:
    void buttonClicked();
    void plotSoleLeft();
    void plotSoleRight();
    void plotKinematics();
    void plotPlantar();
    void plotDorsi();
    void updateGaitPhaseState();
    void dorsiZeroingDone();

private:
    afo_gui::QNode qnode;
    Ui::MainWindow *ui;
    int logNum = 0;
    int max_log = 20;
    int voltPlotMaxNum = 45;
    int motorPlotMaxNum = 45;
    int gaitPhasePlotMaxNum = 50;
    bool is_plot_data = false;
    bool is_plot_sole = false;
    bool is_plantar_run = false;
    bool is_dorsi_run = false;
    bool is_trial_on = false;
    bool is_left_calib_on = false;
    bool is_right_calib_on = false;
    double t_left_calib = 0;
    double t_right_calib = 0;
    double max_torque = 0.3;
    double *linkX, *linkY, *linkZ;
    int currentLink = -1;
    int currentPage = 0;

    double cycle_time = 1.0;

    QVector<double> t_v_l, t_v_r, t_m_p, t_m_d, t_gp;
    QVector<double> v_l[6];
    QVector<double> v_r[6];
    QVector<double> m_p[2];
    QVector<double> m_d[4];
    QVector<double> gp[2];

    double state_gp[2];
    Draw *s_l[6], *s_r[6];
    QVBoxLayout *n_l[6], *n_r[6];
    QCPItemLine *link[7];
    int *rgb;

};


void appendCropQVector(QVector<double> *vector, double data, int maxNum);

#endif // MAINWINDOW_H
