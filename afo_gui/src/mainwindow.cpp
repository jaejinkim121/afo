
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc, argv)
    , ui(new Ui::MainWindow)
{
    qnode.init();
    ui->setupUi(this);
    
    // Load sole image
    QPixmap soleImage("/home/srbl/catkin_ws/src/afo/afo_gui/img_sole.png");
    ui->label_sole_image->setPixmap(soleImage.scaled(300, 400));
    rgb = new int[3];

    // Connect QObject to ui objects.
    //QObject::connect(ui->test, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_plot_data, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_plot_sole, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_sole_calibration_left, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_sole_calibration_right, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_max_torque, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_cycle_time, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_clear_max_torque, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_clear_cycle_time, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_run_dorsi, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_run_plantar, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_streaming, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key1, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key2, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key3, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key4, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key5, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key6, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key7, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key8, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key9, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key0, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key_dot, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_max_torque_key_delete, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key1, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key2, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key3, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key4, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key5, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key6, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key7, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key8, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key9, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key0, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key_dot, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_cycle_time_key_delete, SIGNAL(clicked()), this, SLOT(buttonClicked()));

    QObject::connect(&qnode, SIGNAL(updateSoleLeft()), this, SLOT(plotSoleLeft()));
    QObject::connect(&qnode, SIGNAL(updateSoleRight()), this, SLOT(plotSoleRight()));
    QObject::connect(&qnode, SIGNAL(updatePlantar()), this, SLOT(plotPlantar()));
    QObject::connect(&qnode, SIGNAL(updateDorsi()), this, SLOT(plotDorsi()));
    QObject::connect(&qnode, SIGNAL(updateGaitPhase()), this, SLOT(updateGaitPhaseState()));
    QObject::connect(&qnode, SIGNAL(doneDorsiZeroing()), this, SLOT(dorsiZeroingDone()));

    initPlot();
    initSolePlot();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::buttonClicked(){
    QPushButton* button = (QPushButton*)sender();
    std::string state;
    state = button->objectName().toStdString();

    if (state == "button_toggle_plot_data"){
        this->togglePlotData();
        this->updateLog("toggle plot data button pressed");
    }
    
    else if (state == "button_toggle_plot_sole"){
        this->togglePlotSole();
        this->updateLog("toggle plot sole button pressed");
    }

    else if (state == "button_sole_calibration_left"){
        this->updateLog("Left sole calibration start");
        this->soleCalibrationLeft();
    }

    else if (state == "button_sole_calibration_right"){
        this->updateLog("Right sole calibration start");
        this->soleCalibrationRight();
    }

    else if (state == "button_sole_calibration_proceed"){
        this->proceedCalibration();
    }

    else if (state == "button_set_max_torque"){
        this->setMaxTorque();
    }

    else if (state == "button_set_cycle_time"){
        this->setCycleTime();
    }

    else if (state == "button_clear_max_torque"){
        ui->text_target_max_torque->clear();
    }

    else if (state == "button_clear_cycle_time"){
        ui->text_target_cycle_time->clear();
    }

    else if (state == "button_run_plantar"){
        this->togglePlantarRun();
    }

    else if (state == "button_run_dorsi"){
        this->toggleDorsiRun();
    }

    else if (state == "button_toggle_streaming"){
        this->toggleStreaming();
    }

    else if (state == "button_max_torque_key1"){
        this->typeMaxTorqueKey('1');
    }
    
    else if (state == "button_max_torque_key2"){
        this->typeMaxTorqueKey('2');
    }

    else if (state == "button_max_torque_key3"){
        this->typeMaxTorqueKey('3');
    }

    else if (state == "button_max_torque_key4"){
        this->typeMaxTorqueKey('4');
    }

    else if (state == "button_max_torque_key5"){
        this->typeMaxTorqueKey('5');
    }

    else if (state == "button_max_torque_key6"){
        this->typeMaxTorqueKey('6');
    }

    else if (state == "button_max_torque_key7"){
        this->typeMaxTorqueKey('7');
    }

    else if (state == "button_max_torque_key8"){
        this->typeMaxTorqueKey('8');
    }

    else if (state == "button_max_torque_key9"){
        this->typeMaxTorqueKey('9');
    }

    else if (state == "button_max_torque_key0"){
        this->typeMaxTorqueKey('0');
    }

    else if (state == "button_max_torque_key_dot"){
        this->typeMaxTorqueKey('.');
    }

    else if (state == "button_max_torque_key_delete"){
        this->deleteMaxTorqueKey();
    }
    
    else if (state == "button_cycle_time_key1"){
        this->typeCycleTimeKey('1');
    }
    
    else if (state == "button_cycle_time_key2"){
        this->typeCycleTimeKey('2');
    }
    
    else if (state == "button_cycle_time_key3"){
        this->typeCycleTimeKey('3');
    }
    
    else if (state == "button_cycle_time_key4"){
        this->typeCycleTimeKey('4');
    }
    
    else if (state == "button_cycle_time_key5"){
        this->typeCycleTimeKey('5');
    }
    
    else if (state == "button_cycle_time_key6"){
        this->typeCycleTimeKey('6');
    }
    
    else if (state == "button_cycle_time_key7"){
        this->typeCycleTimeKey('7');
    }
    
    else if (state == "button_cycle_time_key8"){
        this->typeCycleTimeKey('8');
    }
    
    else if (state == "button_cycle_time_key9"){
        this->typeCycleTimeKey('9');
    }
    
    else if (state == "button_cycle_time_key0"){
        this->typeCycleTimeKey('0');
    }
    
    else if (state == "button_cycle_time_key_dot"){
        this->typeCycleTimeKey('.');
    }
    
    else if (state == "button_cycle_time_key_delete"){
        this->deleteCycleTimeKey();
    }
    
}

void MainWindow::togglePlotData(){
    is_plot_data = !is_plot_data;
    if (!is_plot_data){
        t_m_p.clear();
        t_m_d.clear();
        for (int i =0; i< 2; i++){
            m_p[i].clear();
        }
        for (int i = 0; i< 4; i++){
            m_d[i].clear();
        }
    }
}

void MainWindow::togglePlotSole(){
    is_plot_sole = !is_plot_sole;
    if (!is_plot_sole){
        t_v_l.clear();
        t_v_r.clear();
        for (int i = 0; i < 6; i++){
            v_l[i].clear();
            v_r[i].clear();
        }
    }
}

void MainWindow::soleCalibrationLeft(){
    double t_sole = ros::Time::now().toSec();
    qnode.pubThreshold(true);
    while(ros::Time::now().toSec() - t_sole < 2.0){
        continue;
    }
}

void MainWindow::soleCalibrationRight(){
    double t_sole = ros::Time::now().toSec();
    qnode.pubThreshold(false);
    while(ros::Time::now().toSec() - t_sole < 2.0){
        continue;
    }
}

void MainWindow::proceedCalibration(){
    return;
}

void MainWindow::setMaxTorque(){
    float t = 1;
    try{
        t = stof(ui->text_target_max_torque->toPlainText().toStdString());
        qnode.pubMaxTorque(t);
        QString s("maximum torque set to ");
        s.append(QString::fromStdString(std::to_string(t)));
        ui->text_target_max_torque->clear();
        updateLog(s);
        return;
    }
    catch(...){
        updateLog("Target Max Torque is empty");

    }
}

void MainWindow::setCycleTime(){
    float t = 1;
    try{
        t = stof(ui->text_target_cycle_time->toPlainText().toStdString());
        qnode.pubCycleTime(t);
        QString s("Cycle time set to ");
        s.append(QString::fromStdString(std::to_string(t)));
        ui->text_target_cycle_time->clear();
        updateLog(s);
        return;
    }
    catch(...){
        updateLog("Target Cycle Time is empty");

    }
}

void MainWindow::togglePlantarRun(){
    is_plantar_run = !is_plantar_run;
    qnode.pubPlantarRun(is_plantar_run);
    if (is_plantar_run){
        ui->button_run_plantar->setStyleSheet("background-color: rgb(255, 0, 0)");
        ui->button_run_plantar->setText("Stop \nPlantar");
    }
    else{
        ui->button_run_plantar->setStyleSheet("background-color: rgb(0, 255, 0)");
        ui->button_run_plantar->setText("Run \nPlantar");
    }
    
}

void MainWindow::toggleDorsiRun(){
    is_dorsi_run = !is_dorsi_run;
    qnode.pubDorsiRun(is_dorsi_run);
    if (is_dorsi_run){
        ui->button_run_dorsi->setStyleSheet("background-color: rgb(255, 0, 0)");
        ui->button_run_dorsi->setText("Stop \nDorsi");
    }
    else{
        ui->button_run_dorsi->setStyleSheet("background-color: rgb(0, 255, 0)");
        ui->button_run_dorsi->setText("Run \ndorsi");
    }
    
}

void MainWindow::toggleStreaming(){
    qnode.pubStreaming();
    updateLog("Toggle Sole Sensor Streaming");
}

void MainWindow::typeMaxTorqueKey(char c){
    QString q;
    q = ui->text_target_max_torque->toPlainText();
    q.append(QChar(c));
    ui->text_target_max_torque->setPlainText(q);
}

void MainWindow::typeCycleTimeKey(char c){
    QString q;
    q = ui->text_target_cycle_time->toPlainText();
    q.append(QChar(c));
    ui->text_target_cycle_time->setPlainText(q);
}

void MainWindow::deleteMaxTorqueKey(){
    QString q;
    q = ui->text_target_max_torque->toPlainText();
    q.remove(q.size() - 1);
    ui->text_target_max_torque->setPlainText(q);
}

void MainWindow::deleteCycleTimeKey(){
    QString q;
    q = ui->text_target_cycle_time->toPlainText();
    q.remove(q.size() - 1);
    ui->text_target_cycle_time->setPlainText(q);

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

void MainWindow::plotSoleLeft(){
    float* data = qnode.getSoleLeftData();
    if (is_plot_data){
        appendCropQVector(&t_gp, data[0], gaitPhasePlotMaxNum);    
        appendCropQVector(&gp[0], state_gp[0], gaitPhasePlotMaxNum);
        appendCropQVector(&gp[1], state_gp[1], gaitPhasePlotMaxNum);
        ui->plot_gaitPhase->xAxis->setRange(t_gp[0], t_gp[0] + 20.0);
        this->updatePlot(GAIT_PHASE);
    }

    if (!is_plot_sole) return;
    appendCropQVector(&t_v_l, data[0], voltPlotMaxNum);
    for (int i = 0; i < 6; i++){
        appendCropQVector(&v_l[i], data[i+1], voltPlotMaxNum);
    }
    ui->plot_sole_left_voltage->xAxis->setRange(t_v_l[0], t_v_l[0]+5.0);
    this->updatePlot(SOLE_LEFT);
    this->updateSolePlot(SOLE_LEFT, data);
}

void MainWindow::plotSoleRight(){
    if (!is_plot_sole) return;
    float* data = qnode.getSoleRightData();
    appendCropQVector(&t_v_r, data[0], voltPlotMaxNum);
    for (int i = 0; i < 6; i++){
        appendCropQVector(&v_r[i], data[i+1], voltPlotMaxNum);
    }
    ui->plot_sole_right_voltage->xAxis->setRange(t_v_r[0], t_v_r[0]+5.0);
    this->updatePlot(SOLE_RIGHT);
    this->updateSolePlot(SOLE_RIGHT, data);
}

void MainWindow::plotPlantar(){
    if(!is_plot_data) return;
    float* data = qnode.getPlantarData();

    appendCropQVector(&t_m_p, data[0], motorPlotMaxNum);
    appendCropQVector(&m_p[0], data[1], motorPlotMaxNum);
    appendCropQVector(&m_p[1], data[2], motorPlotMaxNum);
    ui->plot_plantar_command->xAxis->setRange(t_m_p[0], t_m_p[0]+5.0);
    this->updatePlot(MOTOR_PLANTAR);
}

void MainWindow::plotDorsi(){
    if (!is_plot_data) return;
    float* data = qnode.getDorsiData();

    appendCropQVector(&t_m_d, data[0], motorPlotMaxNum);
    appendCropQVector(&m_d[0], data[1], motorPlotMaxNum);
    appendCropQVector(&m_d[1], data[2], motorPlotMaxNum);
    appendCropQVector(&m_d[2], data[3], motorPlotMaxNum);
    appendCropQVector(&m_d[3], data[4], motorPlotMaxNum);
    ui->plot_dorsi_command->xAxis->setRange(t_m_d[0], t_m_d[0]+5.0);
    this->updatePlot(MOTOR_DORSI);
}

void MainWindow::updateGaitPhaseState(){
    if(!is_plot_data) return;
    float* data = qnode.getGaitPhase();

    state_gp[0] = data[1] - 1;
    state_gp[1] = data[2] - 1;
}

void MainWindow::dorsiZeroingDone(){
    ui->button_dorsi_zeroing_done->setStyleSheet("background-color: rgb(0, 200, 0)");
    updateLog("Dorsi Zeroing Done");
}

void MainWindow::initPlot(){
    state_gp[0] = 0.0;
    state_gp[1] = 0.0;

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
        ui->plot_sole_right_voltage->graph(i)->setPen(pen[i]);
        ui->plot_sole_left_voltage->graph(i)->setName(QString(char(i)+'1'));
        ui->plot_sole_right_voltage->graph(i)->setName(QString(char(i)+'1'));
        
    }
    ui->plot_sole_left_voltage->yAxis->setRange(0.5, 1.2);
    ui->plot_sole_right_voltage->yAxis->setRange(0.5, 1.2);

    for (int i = 0; i< 4; i++){
        ui->plot_dorsi_command->addGraph();
        ui->plot_dorsi_command->graph(i)->setPen(pen[i]);   
    }
    ui->plot_dorsi_command->graph(0)->setName(QString("tor_des"));
    ui->plot_dorsi_command->graph(1)->setName(QString("tor_act"));
    ui->plot_dorsi_command->graph(2)->setName(QString("pos_des"));
    ui->plot_dorsi_command->graph(3)->setName(QString("pos_act"));
    ui->plot_dorsi_command->yAxis->setRange(-1.2, 1.2);

    ui->plot_plantar_command->addGraph();
    ui->plot_plantar_command->addGraph();
    ui->plot_plantar_command->graph(0)->setPen(pen[0]);
    ui->plot_plantar_command->graph(1)->setPen(pen[1]);
    ui->plot_plantar_command->graph(0)->setName(QString("tor_des"));
    ui->plot_plantar_command->graph(1)->setName(QString("tor_act"));

    ui->plot_plantar_command->yAxis->setRange(-1.2, 1.2);

    ui->plot_gaitPhase->addGraph();
    ui->plot_gaitPhase->addGraph();
    ui->plot_gaitPhase->graph(0)->setPen(pen[0]);
    ui->plot_gaitPhase->graph(1)->setPen(pen[1]);
    ui->plot_gaitPhase->graph(0)->setName("P");
    ui->plot_gaitPhase->graph(1)->setName("NP");
    ui->plot_gaitPhase->yAxis->setRange(0, 1.2);

    // Set plot title
    ui->plot_sole_left_voltage->plotLayout()->insertRow(0);
    ui->plot_sole_right_voltage->plotLayout()->insertRow(0);
    ui->plot_plantar_command->plotLayout()->insertRow(0);
    ui->plot_dorsi_command->plotLayout()->insertRow(0);
    ui->plot_gaitPhase->plotLayout()->insertRow(0);

    QCPTextElement *title_sole_left_voltage = new QCPTextElement(ui->plot_sole_left_voltage, "L Sole", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_sole_right_voltage = new QCPTextElement(ui->plot_sole_right_voltage, "R Sole", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_plantar_command = new QCPTextElement(ui->plot_plantar_command, "Plnatar", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_dorsi_command = new QCPTextElement(ui->plot_dorsi_command, "Dorsi", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_gaitPhase = new QCPTextElement(ui->plot_gaitPhase, "Gait Phase", QFont("sans", 16, QFont::Bold));
    
    ui->plot_sole_left_voltage->plotLayout()->addElement(0, 0, title_sole_left_voltage);
    ui->plot_sole_right_voltage->plotLayout()->addElement(0, 0, title_sole_right_voltage);
    ui->plot_plantar_command->plotLayout()->addElement(0, 0, title_plantar_command);
    ui->plot_dorsi_command->plotLayout()->addElement(0, 0, title_dorsi_command);
    ui->plot_gaitPhase->plotLayout()->addElement(0, 0, title_gaitPhase);

    ui->plot_sole_left_voltage->legend->setVisible(true);
    ui->plot_sole_right_voltage->legend->setVisible(true);
    ui->plot_plantar_command->legend->setVisible(true);
    ui->plot_dorsi_command->legend->setVisible(true);
    ui->plot_gaitPhase->legend->setVisible(true);

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
    
    else if (dataType == GAIT_PHASE){
        ui->plot_gaitPhase->graph(0)->setData(t_gp, gp[0]);
        ui->plot_gaitPhase->graph(1)->setData(t_gp, gp[1]);
        ui->plot_gaitPhase->replot();
    }
}

void MainWindow::initSolePlot(){
    for (int i = 0 ; i < 6; i++){
        s_l[i] = new Draw();
        s_r[i] = new Draw();            
        s_l[i]->setGeometry(20, 10, 300, 400);
        s_r[i]->setGeometry(20, 10, 300, 400);

        n_l[i] = new QVBoxLayout();
        n_l[i]->setSpacing(0);
        n_l[i]->setContentsMargins(0, 0, 0, 0);
        n_l[i]->addWidget(s_l[i]);
        n_r[i] = new QVBoxLayout(); 
        n_r[i]->setSpacing(0);
        n_r[i]->setContentsMargins(0, 0, 0, 0);
        n_r[i]->addWidget(s_r[i]);
    }
    
    ui->label_sole_image_left_1->setLayout(n_l[0]);
    ui->label_sole_image_left_2->setLayout(n_l[1]);
    ui->label_sole_image_left_3->setLayout(n_l[2]);
    ui->label_sole_image_left_4->setLayout(n_l[3]);
    ui->label_sole_image_left_5->setLayout(n_l[4]);
    ui->label_sole_image_left_6->setLayout(n_l[5]);
    ui->label_sole_image_right_1->setLayout(n_r[0]);
    ui->label_sole_image_right_2->setLayout(n_r[1]);
    ui->label_sole_image_right_3->setLayout(n_r[2]);
    ui->label_sole_image_right_4->setLayout(n_r[3]);
    ui->label_sole_image_right_5->setLayout(n_r[4]);
    ui->label_sole_image_right_6->setLayout(n_r[5]);

    s_l[0]->setShape(40, 180, 15, 15);
    s_l[1]->setShape(80, 70, 15, 15);
    s_l[2]->setShape(75, 145, 15, 15);
    s_l[3]->setShape(120, 80, 15, 15);
    s_l[4]->setShape(120, 140, 15, 15);
    s_l[5]->setShape(70, 350, 15, 15);
    s_r[0]->setShape(260, 80, 15, 15);
    s_r[1]->setShape(220, 70, 15, 15);
    s_r[2]->setShape(225, 145, 15, 15);
    s_r[3]->setShape(180, 80, 15, 15);
    s_r[4]->setShape(180, 140, 15, 15);
    s_r[5]->setShape(230, 350, 15, 15);
}

void MainWindow::updateSolePlot(int side, float* data){
    if (side == SOLE_LEFT){
        for (int i = 0; i < 6; i++){
            updateRGB(data[i+1]);
            s_l[i]->redraw(rgb[0], rgb[1], rgb[2]);
        }
    }
    else{
        for (int i = 0; i < 6; i++){
            updateRGB(data[i+1]);
            s_r[i]->redraw(rgb[0], rgb[1], rgb[2]);
        }
    }
}



void MainWindow::updateRGB(float f){
    float a = f/0.25;
    int x = floor(a);
    float y = floor(255*(a-(float)x));
    
    switch(x){
        case 0 : rgb[0] = 255; rgb[1] = y; rgb[2] = 0; break;
        case 1 : rgb[0] = 255-y; rgb[1] = 255; rgb[2] = 0; break;
        case 2 : rgb[0] = 0; rgb[1] = 255; rgb[2] = y; break;
        case 3 : rgb[0] = 0; rgb[1] = 255-y; rgb[2] = 255; break;
        case 4 : rgb[0] = 0; rgb[1] = 0; rgb[2] = 255; break;
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

