
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent, int argc, char** argv)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Connect QObject to ui objects.
    QObject::connect(ui->test, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_plot, SIGNAL(clicked()), this, SLOT(buttonClicked()));

    is = true;
    ros::init(argc, argv, "afo_gui");
    int rr;

    // Set parameters related to ros
    nh.getParam("/rr", rr);
    ros::Rate loop_rate(rr);
    t_begin = ros::Time::now().toSec();

    // Declare publisher and subscriber
    afo_gui_sole_calibration_pub = nh.advertise<std_msgs::Bool>("/afo_gui/soleCalibration", 100);
    afo_gui_max_torque_pub = nh.advertise<std_msgs::Float32>("/afo_gui/max_torque", 100);
    afo_soleSensor_left_sub = nh.subscribe("/afo_sensor/soleSensor_left", 1, this->callbackSoleLeft);
    afo_soleSensor_right_sub = nh.subscribe("/afo_sensor/soleSensor_right", 1, this->callbackSoleRight);
    afo_plantar_command_sub = nh.subscribe("/afo_controller/motor_data_plantar", 1, this->callbackPlantar);
    afo_dorsi_command_sub = nh.subscribe("/afo_controller/motor_data_dorsi", 1, this->callbackDorsi);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::buttonClicked(){
    QPushButton* button = (QPushButton*)sender();
    std::string state;
    state = button->objectName().toStdString();
    if (state == "button_toggle_plot"){
        this->toggleVoltPlot();
        this->updateLog("toggle plot pressed");
    }


//    QVector<double> x(101), y(101); // initialize with entries 0..100
//    for (int i=0; i<101; ++i)
//    {
//        x[i] = i/50.0 - 1; // x goes from -1 to 1
//        y[i] = x[i]*x[i] + offset; // let's plot a quadratic function
//    }
//    ui->plot->addGraph();
//    ui->plot->graph(0)->setData(x,y);
//    ui->plot->xAxis->setLabel("x");
//    ui->plot->yAxis->setLabel("y");
//    // set axes ranges, so we see all data:
//    ui->plot->xAxis->setRange(-1, 1);
//    ui->plot->yAxis->setRange(0, 15);
//    ui->plot->replot();
//    offset++;
}

void MainWindow::toggleVoltPlot(){
    is_plot = !is_plot;
    if (!is_plot){
        t_v_l.clear();
        t_v_r.clear();
        for (int i = 0; i < 6; i++){
            v_l[i].clear();
            v_r[i].clear();
        }
    }
}


void MainWindow::updateLog(QString s){
    if (logNum > max_log){
        QTextCursor cursor = ui->log->textCursor();

        cursor.movePosition(QTextCursor::Start);
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, 1);
        cursor.select(QTextCursor::LineUnderCursor);
        cursor.removeSelectedText();
        cursor.deleteChar(); // clean up new line
        cursor.movePosition(QTextCursor::End);
        ui->log->setTextCursor(cursor);
    }
    else {
        logNum++;
    }
    ui->log->append(s);
    ui->log->update();
}

void MainWindow::set_emergency(bool on){
    if (on){
        ui->button_emergency_stop->setStyleSheet("background-color: rgb(200, 200, 200)");
    }
    else{
        ui->button_emergency_stop->setStyleSheet("background-color: rgb(255, 0, 0)");
    }
}

void MainWindow::callbackSoleLeft(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if (!is_plot) return;

    double t = ros::Time::now().toSec() - t_begin;

    appendCropQVector(&t_v_l, t, voltPlotMaxNum);

    for (int i = 0; i < 6; i++){
        appendCropQVector(&v_l[i], msg->data[i], voltPlotMaxNum);
    }
    this->updatePlot(SOLE_LEFT);
}

void MainWindow::callbackSoleRight(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if (!is_plot) return;

    double t = ros::Time::now().toSec() - t_begin;

    appendCropQVector(&t_v_r, t, voltPlotMaxNum);

    for (int i = 0; i < 6; i++){
        appendCropQVector(&v_r[i], msg->data[i], voltPlotMaxNum);
    }

    this->updatePlot(SOLE_RIGHT);
}

void MainWindow::callbackPlantar(const std_msgs::Float32MultiArray::ConstPtr& msg){
    double t = ros::Time::now().toSec() - t_begin;

    appendCropQVector(&t_m_p, t, motorPlotMaxNum);
    appendCropQVector(&m_p[0], msg->data[2], motorPlotMaxNum);
    appendCropQVector(&m_p[1], msg->data[5], motorPlotMaxNum);

    this->updatePlot(MOTOR_PLANTAR);
}

void MainWindow::callbackDorsi(const std_msgs::Float32MultiArray::ConstPtr& msg){
    double t = ros::Time::now().toSec() - t_begin;

    appendCropQVector(&t_m_d, t, motorPlotMaxNum);
    appendCropQVector(&m_d[0], msg->data[2], motorPlotMaxNum);
    appendCropQVector(&m_d[1], msg->data[5], motorPlotMaxNum);
    appendCropQVector(&m_d[2], msg->data[3], motorPlotMaxNum);
    appendCropQVector(&m_d[3], msg->data[6], motorPlotMaxNum);

    this->updatePlot(MOTOR_DORSI);
}

void MainWindow::initPlot(){
    QPen pen[6];
    pen[0].setColor(QColor(0,0,0));
    pen[1].setColor(QColor(255, 0, 0));
    pen[2].setColor(QColor(0, 0, 255));
    pen[3].setColor(QColor(0, 255, 0));
    pen[4].setColor(QColor(255, 165, 0));
    pen[5].setColor(QColor(198, 115, 255));

    for(int i = 0; i< 6; i++){
        ui->plot_sole_left_voltage->addGraph();
        ui->plot_sole_right_voltage->addGraph();
        ui->plot_sole_left_voltage->graph(i)->setPen(pen[i]);
    }

    for (int i = 0; i< 4; i++){
        ui->plot_dorsi_command->addGraph();
        ui->plot_dorsi_command->graph(i)->setPen(pen[i]);
    }
    ui->plot_plantar_command->addGraph();
    ui->plot_plantar_command->addGraph();
    ui->plot_plantar_command->graph(0)->setPen(pen[0]);
    ui->plot_plantar_command->graph(1)->setPen(pen[1]);

}

void MainWindow::updatePlot(int dataType){
    if (dataType == SOLE_LEFT){
        for(int i = 0; i < 6; i++){
            ui->plot_sole_left_voltage->graph(i)->setData(t_v_l, v_l[i]);
        }
        ui->plot_sole_left_voltage->replot();
    }

    else if (dataType == SOLE_RIGHT){
        for(int i = 0; i < 6; i++){
            ui->plot_sole_right_voltage->graph(i)->setData(t_v_r, v_r[i]);
        }
        ui->plot_sole_right_voltage->replot();
    }

    else if (dataType == MOTOR_PLANTAR){
        ui->plot_plantar_command->graph(0)->setData(t_m_p, m_p[0]);
        ui->plot_plantar_command->graph(1)->setData(t_m_p, m_p[1]);
        ui->plot_plantar_command->replot();
    }

    else if (dataType == MOTOR_DORSI){
        for (int i = 0; i< 4; i++){
            ui->plot_dorsi_command->graph(i)->setData(t_m_d, m_d[i]);
        }
        ui->plot_dorsi_command->replot();
    }

}

void appendCropQVector(QVector<double> *vector, double data, int maxNum){
    if (maxNum == 0){
        return;
    }

    if(vector->size() < maxNum){
        vector->append(data);
    }
    else{
        vector -> remove(0);
        vector -> append(data);
    }
    return;
}


