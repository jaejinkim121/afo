
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
    void setMaxTorque(bool is_plantar);
    void setRiseTime(bool is_plantar);
    void setFallTime(bool is_plantar);
    void setTriggerTime(bool is_plantar);
    void setCycleTime();
    void setLinkLength();
    void setPFO();
    void setPIC();
    void setNFO();
    void setNIC();
    void setThreshold();
    void updatePlotThreshold();
    void toggleAffectedSide();
    void togglePlantarRun();
    void toggleDorsiRun();
    void toggleStreaming();
    void typeKey(char c);
    void deleteKey();
    void toggleTrial();
    void emergencyStop();
    void updateMaxTorqueValue(bool is_plantar);
    void updateRiseTimeValue(bool is_plantar);
    void updateFallTimeValue(bool is_plantar);
    void updateTriggerTimeValue(bool is_plantar);
    void updateCycleTimeValue();
    void targetLinkIdx();
    void imuZero();
    void togglePage(bool page);
    void sendSync();
    void runPolycalibZero();
    void runPolycalib();
    void polyCalibToggle(bool forward);
    void polyCalibSide();
    bool polyCalibNum(bool forward);
    bool polyCalibForce(bool forward);
    void setSessionType(
        unsigned int type_trial, 
        unsigned int type_control, 
        unsigned int type_cue
        );
    
    // MH
    void runPFMH();
    void runDFMH();

    
    void updateLog(QString s);
    void updateParameterFile();
    void loadParameterFile();
    void set_emergency(bool on);
    
    void updatePlot(int dataType);
    void initPlot();
    void initSolePlot();
    void updateSolePlot(int side,  float* data);
    void updateRGB(float f);
    void updatePolyFit();

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
    int gaitPhasePlotMaxNum2 = 250;
    bool is_plot_data = false;
    bool is_plot_sole = false;
    bool is_plantar_run = false;
    bool is_dorsi_run = false;
    bool is_trial_on = false;
    bool is_left_calib_on = false;
    bool is_right_calib_on = false;
    bool sync = false;
    double t_left_calib = 0;
    double t_right_calib = 0;
    double *linkX, *linkY, *linkZ;
    int currentLink = -1;
    int current_affected_side = 1;
    int poly_side = 1;
    int poly_num = 1;
    int poly_force = 1;

    double cycle_time = 1.4;
    double max_torque[2] = {0.3, 0.2};
    double rise_time[2] = {0.2, 0.2};
    double fall_time[2] = {0.2, 0.05};
    double trigger_time[2] = {0.2, 0.05};
    float threshold[4] = {3, 6, 3, 6};
//    float threshold[4] = {0.01, 0.02, 0.01, 0.02};

    QVector<double> t_v_l, t_v_r, t_m_p, t_m_d, t_gp, t_gp2;
    QVector<double> v_l[6];
    QVector<double> v_r[6];
    QVector<double> m_p[2];
    QVector<double> m_d[4];
    QVector<double> gp[2], gp2[2];

    double state_gp[2];
    Draw *s_l[6], *s_r[6];
    QVBoxLayout *n_l[6], *n_r[6];
    QCPItemLine *link[7];
    QCPItemStraightLine *infLineThreshold[4];
    int *rgb;

};


void appendCropQVector(QVector<double> *vector, double data, int maxNum);
std::string CutOnDecimalPt(std::string num, int point);
#endif // MAINWINDOW_H
