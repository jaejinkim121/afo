
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
    QPixmap soleImage("/home/afo/catkin_ws/src/afo/afo_gui/img_sole.png");
    ui->label_sole_image->setPixmap(soleImage.scaled(300, 400));
    rgb = new int[3];

    // Connect QObject to ui objects.
    QObject::connect(ui->button_toggle_plot_data, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_plot_sole, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_sole_calibration_left, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_sole_calibration_right, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_max_torque_plantar, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_max_torque_dorsi, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_rise_time_plantar, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_rise_time_dorsi, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_fall_time_plantar, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_fall_time_dorsi, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_trigger_time_plantar, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_trigger_time_dorsi, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_cycle_time, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_pfo, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_pic, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_nfo, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_nic, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_hic, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_hfo, SIGNAL(clicked()), this, SLOT(buttonClicked()));

    QObject::connect(ui->button_set_threshold, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_affected_side, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_clear_parameter, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_run_dorsi, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_run_plantar, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_streaming, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key1, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key2, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key3, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key4, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key5, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key6, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key7, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key8, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key9, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key0, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key_dot, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_key_delete, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_trial, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_emergency, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_imu_zero, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_link_length, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_target_link_idx, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_page, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_param_toggle_page, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_sync, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_polycalib_zero_run, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_polycalib_run, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_polycalib_reroll, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_polycalib_skip, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_polycalib_side, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_polycalib_num, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_polycalib_force, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_10bx, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_10xx, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_10ox, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_10bo, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_10xo, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_10xx, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_2bx, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_2xx, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_2ox, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_2bo, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_2xo, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_2oo, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_set_session_na, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_forced_trigger, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_run_pf_mh, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_run_df_mh, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_run_both_mh, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_plot_on_woc, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_flush, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    
    
    QObject::connect(&qnode, SIGNAL(updateSoleLeft()), this, SLOT(plotSoleLeft()));
    QObject::connect(&qnode, SIGNAL(updateSoleRight()), this, SLOT(plotSoleRight()));
    QObject::connect(&qnode, SIGNAL(updateKinematics()), this, SLOT(plotKinematics()));
    QObject::connect(&qnode, SIGNAL(updatePlantar()), this, SLOT(plotPlantar()));
    QObject::connect(&qnode, SIGNAL(updateDorsi()), this, SLOT(plotDorsi()));
    QObject::connect(&qnode, SIGNAL(updateGaitPhase()), this, SLOT(updateGaitPhaseState()));
    QObject::connect(&qnode, SIGNAL(updatePolyFitPlot()), this, SLOT(updatePolyFit()));
    QObject::connect(&qnode, SIGNAL(doneDorsiZeroing()), this, SLOT(dorsiZeroingDone()));
    QObject::connect(&qnode, SIGNAL(updateTLA()), this, SLOT(plotTLA()));
    QObject::connect(&qnode, SIGNAL(updateWOC()), this, SLOT(plotWOC()));
    ui->RightBox->setCurrentIndex(0);
    ui->tabWidget->setCurrentIndex(0);

    loadParameterFile();
    initPlot();
    initSolePlot();
    toggleTrial();
    toggleTrial();
    ui->horSlider_affected_side->setSliderPosition(current_affected_side);
    updateMaxTorqueValue(true);
    updateMaxTorqueValue(false);
    updateRiseTimeValue(true);
    updateRiseTimeValue(false);
    updateFallTimeValue(true);
    updateFallTimeValue(false);
    updateTriggerTimeValue(true);
    updateTriggerTimeValue(false);
    updateCycleTimeValue();
    setPFO();
    setPIC();
    setNFO();
    setNIC();
    setHIC();
    setHFO();
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

    else if (state == "button_set_max_torque_plantar"){
        this->setMaxTorque(true);
        this->updateParameterFile();
    }

    else if (state == "button_set_max_torque_dorsi"){
        this->setMaxTorque(false);
        this->updateParameterFile();

    }

    else if (state == "button_set_rise_time_plantar"){
        this->setRiseTime(true);
        this->updateParameterFile();

    }

    else if (state == "button_set_rise_time_dorsi"){
        this->setRiseTime(false);
        this->updateParameterFile();
    }

    else if (state == "button_set_fall_time_plantar"){
        this->setFallTime(true);
        this->updateParameterFile();
    }

    else if (state == "button_set_fall_time_dorsi"){
        this->setFallTime(false);
        this->updateParameterFile();
    }

    else if (state == "button_set_trigger_time_plantar"){
        this->setTriggerTime(true);
        this->updateParameterFile();
    }

    else if (state == "button_set_trigger_time_dorsi"){
        this->setTriggerTime(false);
        this->updateParameterFile();
    }

    else if (state == "button_set_cycle_time"){
        this->setCycleTime();
        this->updateParameterFile();
    }

    else if (state == "button_set_pfo"){
        this->setPFO();
        this->updateParameterFile();
    }

    else if (state == "button_set_pic"){
        this->setPIC();
        this->updateParameterFile();
    }
    else if (state == "button_set_hic"){
        this->setHIC();
        this->updateParameterFile();
    }
    else if (state == "button_set_hfo"){
        this->setHFO();
        this->updateParameterFile();
    }
    else if (state == "button_set_nfo"){
        this->setNFO();
        this->updateParameterFile();
    }

    else if (state == "button_set_nic"){
        this->setNIC();
        this->updateParameterFile();
    }

    else if (state == "button_set_threshold"){
        this->setThreshold();
        this->updateParameterFile();
    }

    else if (state == "button_affected_side"){
        this->toggleAffectedSide();
        this->updateParameterFile();
    }

    else if (state == "button_clear_parameter"){
        ui->text_target_parameter->clear();
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

    else if (state == "button_key1"){
        this->typeKey('1');
    }
    
    else if (state == "button_key2"){
        this->typeKey('2');
    }

    else if (state == "button_key3"){
        this->typeKey('3');
    }

    else if (state == "button_key4"){
        this->typeKey('4');
    }

    else if (state == "button_key5"){
        this->typeKey('5');
    }

    else if (state == "button_key6"){
        this->typeKey('6');
    }

    else if (state == "button_key7"){
        this->typeKey('7');
    }

    else if (state == "button_key8"){
        this->typeKey('8');
    }

    else if (state == "button_key9"){
        this->typeKey('9');
    }

    else if (state == "button_key0"){
        this->typeKey('0');
    }

    else if (state == "button_key_dot"){
        this->typeKey('.');
    }

    else if (state == "button_key_delete"){
        this->deleteKey();
    }

    else if (state == "button_toggle_trial"){
        this->toggleTrial();
    }

    else if (state == "button_emergency"){
        this->emergencyStop();
    }

    else if (state == "button_imu_zero"){
        this->imuZero();
    }

    else if (state == "button_set_link_length"){
        this->setLinkLength();
    }
    
    else if (state == "button_target_link_idx"){
        this->targetLinkIdx();
    }

    else if (state == "button_toggle_page"){
        this->togglePage(true);
    }

    else if (state == "button_param_toggle_page"){
        this->togglePage(false);
    }

    else if (state == "button_sync"){
        this->sendSync();
    }
    
    else if (state == "button_polycalib_zero_run"){
        this->runPolycalibZero();
    }

    else if (state == "button_polycalib_run"){
        this->runPolycalib();
    }

    else if (state == "button_polycalib_reroll"){
        this->polyCalibToggle(false);
    }

    else if (state == "button_polycalib_skip"){
        this->polyCalibToggle(true);
    }

    else if (state == "button_polycalib_side"){
        this->polyCalibSide();
    }

    else if (state == "button_polycalib_num"){
        this->polyCalibNum(true);
    }

    else if (state == "button_polycalib_force"){
        this->polyCalibForce(true);
    }
    
    else if (state == "button_set_session_10bx"){
        this->setSessionType(0, 0, 0);
    }
    else if (state == "button_set_session_10xx"){
        this->setSessionType(0, 1, 0);
    }
    else if (state == "button_set_session_10ox"){
        this->setSessionType(0, 2, 0);
    }
    else if (state == "button_set_session_10bo"){
        this->setSessionType(0, 0, 1);
    }
    else if (state == "button_set_session_10xo"){
        this->setSessionType(0, 1, 1);
    }
    else if (state == "button_set_session_10oo"){
        this->setSessionType(0, 2, 1);
    }
    else if (state == "button_set_session_2bx"){
        this->setSessionType(1, 0, 0);
    }
    else if (state == "button_set_session_2xx"){
        this->setSessionType(1, 1, 0);
    }
    else if (state == "button_set_session_2ox"){
        this->setSessionType(1, 2, 0);
    }
    else if (state == "button_set_session_2bo"){
        this->setSessionType(1, 0, 1);
    }
    else if (state == "button_set_session_2xo"){
        this->setSessionType(1, 1, 1);
    }
    else if (state == "button_set_session_2oo"){
        this->setSessionType(1, 2, 1);
    }
    else if (state == "button_set_session_na"){
        this->setSessionType(2, 0, 0);
    }
    else if (state == "button_run_pf_mh"){
        runPFMH();
    }
    else if (state == "button_run_df_mh"){
        runDFMH();
    }
    else if (state == "button_run_both_mh"){
        runPFMH();
        runDFMH();
    }
    else if (state == "button_forced_trigger"){
        qnode.pubForcedTrigger();
    }
    else if (state == "button_plot_on_woc"){
        this->is_plot_woc = !(this->is_plot_woc);
    }
    else if (state == "button_flush"){
        qnode.pubFlush();
    }

}
void MainWindow::runPFMH(){
    qnode.pubRunPFMH();
}
void MainWindow::runDFMH(){
    qnode.pubRunDFMH();
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
    is_left_calib_on = true;
    t_left_calib = ros::Time::now().toSec();
    qnode.pubThresholdGap(this->threshold);
    qnode.pubThreshold(true);
    ui->button_sole_calibration_left->setText("On...");
    ui->button_sole_calibration_left->setStyleSheet("background-color: rgb(255, 0, 0)");

}

void MainWindow::soleCalibrationRight(){
    is_right_calib_on = true;
    t_right_calib = ros::Time::now().toSec();
    qnode.pubThresholdGap(this->threshold);
    qnode.pubThreshold(false);
    ui->button_sole_calibration_right->setText("On...");
    ui->button_sole_calibration_right->setStyleSheet("background-color: rgb(255, 0, 0)");
}

void MainWindow::setMaxTorque(bool is_plantar){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        
        if(is_plantar) max_torque[0] = t;
        else max_torque[1] = t;

        qnode.pubMaxTorque(max_torque[0], max_torque[1]);

        ui->text_target_parameter->clear();

        updateMaxTorqueValue(is_plantar);
        return;
    }
    catch(...){
        updateLog("Target Max Torque is empty");
    }
}

void MainWindow::setRiseTime(bool is_plantar){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        
        if(is_plantar) rise_time[0] = t;
        else rise_time[1] = t;

        qnode.pubRiseTime(rise_time[0], rise_time[1]);

        ui->text_target_parameter->clear();

        updateRiseTimeValue(is_plantar);
        return;
    }
    catch(...){
        updateLog("Target Max Torque is empty");
    }
}

void MainWindow::setFallTime(bool is_plantar){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        
        if(is_plantar) fall_time[0] = t;
        else fall_time[1] = t;

        qnode.pubFallTime(fall_time[0], fall_time[1]);

        ui->text_target_parameter->clear();

        updateFallTimeValue(is_plantar);
        return;
    }
    catch(...){
        updateLog("Target Max Torque is empty");
    }
}
void MainWindow::setTriggerTime(bool is_plantar){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        
        if(is_plantar) trigger_time[0] = t;
        else trigger_time[1] = t;

        qnode.pubTriggerTime(trigger_time[0], trigger_time[1]);

        ui->text_target_parameter->clear();

        updateTriggerTimeValue(is_plantar);
        return;
    }
    catch(...){
        updateLog("Target Max Torque is empty");
    }
}

void MainWindow::setCycleTime(){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        cycle_time = t;
        qnode.pubCycleTime(t);

        ui->text_target_parameter->clear();

        QString s("Cycle time set to ");
        s.append(QString::fromStdString(std::to_string(t)));

        updateCycleTimeValue();
        updateLog(s);
        return;
    }
    catch(...){
        updateLog("Target Cycle Time is empty");

    }
}

void MainWindow::setLinkLength(){
    if (currentLink == -1){
        return;
    }
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        qnode.updateLinkLength(this->currentLink, t);
        ui->text_target_parameter->clear();
    }
    catch(...){
        updateLog("Target link length is wrong");
    }
}

void MainWindow::setPFO(){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        threshold[2] = t;

        ui->text_target_parameter->clear();
        }
    catch(...){
        updateLog("Target is empty");
    }
    std::string s = "PFO\n";
    s.append(CutOnDecimalPt(std::to_string(threshold[2]), 2));
    ui->button_set_pfo->setText(QString::fromStdString(s));
}
void MainWindow::setHIC(){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        threshold[4] = t;

        ui->text_target_parameter->clear();
        }
    catch(...){
        updateLog("Target is empty");
    }
    std::string s = "HIC\n";
    s.append(CutOnDecimalPt(std::to_string(threshold[4]), 2));
    ui->button_set_hic->setText(QString::fromStdString(s));
}
void MainWindow::setHFO(){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        threshold[5] = t;

        ui->text_target_parameter->clear();
        }
    catch(...){
        updateLog("Target is empty");
    }
    std::string s = "HFO\n";
    s.append(CutOnDecimalPt(std::to_string(threshold[5]), 2));
    ui->button_set_hfo->setText(QString::fromStdString(s));
}
void MainWindow::setPIC(){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        threshold[3] = t;

        ui->text_target_parameter->clear();
        }
    catch(...){
        updateLog("Target is empty");
    }
    std::string s = "PIC\n";
    s.append(CutOnDecimalPt(std::to_string(threshold[3]), 2));
    ui->button_set_pic->setText(QString::fromStdString(s));
    
}

void MainWindow::setNFO(){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        threshold[0] = t;

        ui->text_target_parameter->clear();
        }
    catch(...){
        updateLog("Target is empty");
    }
    std::string s = "NFO\n";
    s.append(CutOnDecimalPt(std::to_string(threshold[0]), 2));
    ui->button_set_nfo->setText(QString::fromStdString(s));
}

void MainWindow::setNIC(){
    float t = 1;
    try{
        t = stof(ui->text_target_parameter->toPlainText().toStdString());
        threshold[1] = t;

        ui->text_target_parameter->clear();
        }
    catch(...){
        updateLog("Target is empty");
    }
    std::string s = "NIC\n";
    s.append(CutOnDecimalPt(std::to_string(threshold[1]), 2));
    ui->button_set_nic->setText(QString::fromStdString(s));
}

void MainWindow::setThreshold(){
    qnode.pubThresholdGap(this->threshold);
    updatePlotThreshold();
}

void MainWindow::updatePlotThreshold(){
    infLineThreshold[0]->point1->setCoords(1, threshold[2 - 2 * current_affected_side]);
    infLineThreshold[0]->point2->setCoords(2, threshold[2 - 2 * current_affected_side]);
    infLineThreshold[1]->point1->setCoords(1, threshold[3 - 2 * current_affected_side]);
    infLineThreshold[1]->point2->setCoords(2, threshold[3 - 2 * current_affected_side]);
    infLineThreshold[2]->point1->setCoords(1, threshold[2 * current_affected_side]);
    infLineThreshold[2]->point2->setCoords(2, threshold[2 * current_affected_side]);
    infLineThreshold[3]->point1->setCoords(1, threshold[1 + 2 * current_affected_side]);
    infLineThreshold[3]->point2->setCoords(2, threshold[1 + 2 * current_affected_side]);
ui->plot_sole_left_voltage->yAxis->setRange(0, threshold[3-current_affected_side]+5.0);
ui->plot_sole_right_voltage->yAxis->setRange(0, threshold[1 + 2 * current_affected_side]+5.0);
    ui->plot_sole_left_voltage->replot();
    ui->plot_sole_right_voltage->replot();
}

void MainWindow::toggleAffectedSide(){
    current_affected_side = 1 - current_affected_side;
    ui->horSlider_affected_side->setSliderPosition(current_affected_side);
    qnode.pubAffectedSide(current_affected_side);
    updatePlotThreshold();
}

void MainWindow::targetLinkIdx(){
    switch(currentLink){
        case -1:
        case 0:
            currentLink++;
            ui->text_target_link_idx->setPlainText("left foot");
            break;
        case 1:
            currentLink++;
            ui->text_target_link_idx->setPlainText("left shank");
            break;
        case 2:
            currentLink++;
            ui->text_target_link_idx->setPlainText("left thigh");
            break;
        case 3:
            currentLink++;
            ui->text_target_link_idx->setPlainText("torso");
            break;
        case 4:
            currentLink++;
            ui->text_target_link_idx->setPlainText("right thigh");
            break;
        case 5:
            currentLink++;
            ui->text_target_link_idx->setPlainText("right shank");
            break;
        case 6:
            currentLink = 0;
            ui->text_target_link_idx->setPlainText("right foot");
            break;
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

void MainWindow::typeKey(char c){
    QString q;
    q = ui->text_target_parameter->toPlainText();
    q.append(QChar(c));
    ui->text_target_parameter->setPlainText(q);
}

void MainWindow::deleteKey(){
    QString q;
    q = ui->text_target_parameter->toPlainText();
    q.remove(q.size() - 1);
    ui->text_target_parameter->setPlainText(q);
}


void MainWindow::toggleTrial(){
    if (this->is_trial_on){
        ui->button_toggle_trial->setStyleSheet("background-color: rgb(211, 211, 211)");
        ui->button_toggle_trial->setText("Run Trial");
        ui->button_emergency->stackUnder(ui->RightBox);
        ui->button_emergency->setStyleSheet("background-color: rgba(255, 255, 255, 0%)");

        if (is_plantar_run) togglePlantarRun();
        if (is_dorsi_run) toggleDorsiRun();
    }
    else{
        ui->button_toggle_trial->setStyleSheet("background-color: rgb(255, 0, 0)");
        ui->button_toggle_trial->setText("Stop Trial");
        ui->RightBox->stackUnder(ui->button_emergency);
        ui->button_emergency->setStyleSheet("color: rgba(255,255,255,40%); background-color: rgba(255, 0, 0, 40%); border:none");

        if (!is_plantar_run) togglePlantarRun();
        if (!is_dorsi_run) toggleDorsiRun();
    }
    this->is_trial_on = !this->is_trial_on;
}

void MainWindow::emergencyStop(){
    if (!is_trial_on) return;
    
    if(is_plantar_run) togglePlantarRun();
    if(is_dorsi_run) toggleDorsiRun();
    
    toggleTrial();
}

void MainWindow::updateMaxTorqueValue(bool is_plantar){
    if (is_plantar){
        std::string s = std::to_string(max_torque[0]);
        s = CutOnDecimalPt(s, 2);
        ui->button_set_max_torque_plantar->setText(QString::fromStdString(s));
    }
    else{
        std::string s = std::to_string(max_torque[1]);
        s = CutOnDecimalPt(s, 2);
        ui->button_set_max_torque_dorsi->setText(QString::fromStdString(s));    
    }
}

void MainWindow::updateRiseTimeValue(bool is_plantar){
    if (is_plantar){
        std::string s = std::to_string(rise_time[0]);
        s = CutOnDecimalPt(s, 2);
        ui->button_set_rise_time_plantar->setText(QString::fromStdString(s));
    }
    else{
        std::string s = std::to_string(rise_time[1]);
        s = CutOnDecimalPt(s, 2);
        ui->button_set_rise_time_dorsi->setText(QString::fromStdString(s));
    }
}

void MainWindow::updateFallTimeValue(bool is_plantar){
    if (is_plantar){
        std::string s = std::to_string(fall_time[0]);
        s = CutOnDecimalPt(s, 2);
        ui->button_set_fall_time_plantar->setText(QString::fromStdString(s));
    }
    else{
        std::string s = std::to_string(fall_time[1]);
        s = CutOnDecimalPt(s, 2);
        ui->button_set_fall_time_dorsi->setText(QString::fromStdString(s));
    }
}

void MainWindow::updateTriggerTimeValue(bool is_plantar){
    if (is_plantar){
        std::string s = std::to_string(trigger_time[0]);
        s = CutOnDecimalPt(s, 2);
        ui->button_set_trigger_time_plantar->setText(QString::fromStdString(s));
    }
    else{
        std::string s = std::to_string(trigger_time[1]);
        s = CutOnDecimalPt(s, 2);
        ui->button_set_trigger_time_dorsi->setText(QString::fromStdString(s));
    }
}

void MainWindow::updateCycleTimeValue(){
    std::string s = std::to_string(cycle_time);
    s = CutOnDecimalPt(s, 2);
    ui->button_set_cycle_time->setText(QString::fromStdString(s));
}

void MainWindow::togglePage(bool page){
    int currentPage;
    if (page){
        currentPage = ui->RightBox->currentIndex();
        
        if(++currentPage == 5){
            currentPage = 0;
        }
        
        ui->RightBox->setCurrentIndex(currentPage);
    }
    else{
        currentPage = ui->tabWidget->currentIndex();
        
        if(++currentPage == 3){
            currentPage = 0;
        }
        
        ui->tabWidget->setCurrentIndex(currentPage);
    }
}

void MainWindow::sendSync(){
    sync = !sync;
    qnode.pubSync(sync);

    if(sync){
        ui->button_sync->setStyleSheet("color: rgb(0, 255, 0)");
    }
    else{
        ui->button_sync->setStyleSheet("color: rgb(211,211, 211)");
    }
}

void MainWindow::runPolycalibZero(){
    qnode.pubPolycalib(3, 0, 0);
    double t_start = ros::Time::now().toSec();
    ui->button_polycalib_zero_run->setText("Wait...");
    ui->button_polycalib_zero_run->setStyleSheet("background-color: rgb(0,255,0)");
    while(ros::Time::now().toSec() - t_start < 1.5){
        continue;
    }
    ui->button_polycalib_zero_run->setText("Done\nZero");
    ui->button_polycalib_zero_run->setStyleSheet("background-color: rgb(0,255,0)");
}

void MainWindow::runPolycalib(){
    qnode.pubPolycalib(poly_side, poly_num, poly_force);
    polyCalibToggle(true);
    double t_start = ros::Time::now().toSec();
    ui->button_polycalib_run->setText("Wait....");
    ui->button_polycalib_run->setStyleSheet("background-color: rgb(0, 255, 0)");
    while(ros::Time::now().toSec() - t_start < 1.5){
        continue;
    }
    ui->button_polycalib_run->setText("Next\nCalib");
    ui->button_polycalib_run->setStyleSheet("background-color: rgb(211, 211, 211)");

    if(poly_side != 0) return;

    qnode.pubPolycalib(poly_side, poly_num, poly_force);
    poly_side = 1;

    ui->button_polycalib_run->setText("Run Calibration");
    
}

void MainWindow::polyCalibToggle(bool forward){
    //if (!polyCalibForce(forward)) return;
    if (!polyCalibNum(forward)) return;
    polyCalibSide();

    if(!forward) return;
    
    if (poly_side == 1 && poly_num == 1 && poly_force == 1){
        poly_side = 0;
    }
}

void MainWindow::polyCalibSide(){
    poly_side = 3 - poly_side;
    ui->horSlider_polycalib_side->setSliderPosition(poly_side - 1);
}

bool MainWindow::polyCalibNum(bool forward){
    bool r = false;
    if (forward){
        poly_num++;
        if (poly_num == 7){
            poly_num = 1;
            r = true;
        }


    }
    else{
        poly_num--;
        if(poly_num == 0){
            poly_num = 6;
            r = true;
        }            
    }

    ui->lcdNum_polycalib_num->display(QString::fromStdString(std::to_string(poly_num)));

    return r;
}

bool MainWindow::polyCalibForce(bool forward){
    bool r = (poly_force == 2);
    poly_force = 3 - poly_force;

    ui->horSlider_polycalib_force->setSliderPosition(poly_force - 1);

    return r;
}

void MainWindow::setSessionType(unsigned int type_trial, unsigned int type_control, unsigned int type_cue)
{
    qnode.pubSessionType(type_trial, type_control, type_cue);
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

void MainWindow::updateParameterFile(){
    std::ofstream f("/home/afo/catkin_ws/src/afo/parameter_list.csv");
    f << max_torque[0] << "\n" 
    << max_torque[1] << "\n"
    << trigger_time[0] << "\n"
    << rise_time[0] << "\n" 
    << fall_time[0] << "\n"
    << flat_time[0] << "\n"
    << trigger_time[1] << "\n"
    << rise_time[1] << "\n"
    << fall_time[1] << "\n"
    << stance_time << "\n"
    << pre_tension[0] << "\n"
    << pre_tension[1] << "\n"
    << cycle_time << "\n"
    << current_affected_side << "\n"
    << threshold[0] << "\n"
    << threshold[1] << "\n"
    << threshold[2] << "\n"
    << threshold[3] << "\n"
    << threshold[4] << "\n"
    << threshold[5];
    f.close();
}

void MainWindow::loadParameterFile(){
    std::ifstream f("/home/afo/catkin_ws/src/afo/parameter_list.csv");
    std::cout << "Loading Params" << std::endl;
    std::string str;
    float params[20];
    for (int i = 0; i<20;i++){
        getline(f, str);
        params[i] = stof(str);
    }
    max_torque[0] = params[0];
    max_torque[1] = params[1];
    trigger_time[0] = params[2];
    rise_time[0] = params[3];
    fall_time[0] = params[4];
    flat_time[0] = params[5];
    trigger_time[1] = params[6];
    rise_time[1] = params[7];
    fall_time[1] = params[8];
    stance_time = params[9];
    pre_tension[0] = params[10];
    pre_tension[1] = params[11];
    cycle_time = params[12];
    current_affected_side = params[13];
    threshold[0] = params[14];
    threshold[1] = params[15];
    threshold[2] = params[16];
    threshold[3] = params[17];
    threshold[4] = params[18];
    threshold[5] = params[19];

    f.close();
    std::cout << "Loading Params ENDDDDD" << std::endl;
}

void MainWindow::set_emergency(bool on){
    if (on){
        ui->button_emergency->setStyleSheet("background-color: rgb(211, 211, 211)");
    }
    else{
        ui->button_emergency->setStyleSheet("background-color: rgb(255, 0, 0)");
    }
}

void MainWindow::imuZero(){
    qnode.imuZeroing();
}

void MainWindow::plotSoleLeft(){
    float* data = qnode.getSoleLeftData();
    if (is_plot_data){
        appendCropQVector(&t_gp, data[0], gaitPhasePlotMaxNum);
        appendCropQVector(&gp[0], state_gp[0], gaitPhasePlotMaxNum);
        appendCropQVector(&gp[1], state_gp[1], gaitPhasePlotMaxNum);
        appendCropQVector(&t_gp2, data[0], gaitPhasePlotMaxNum2);
        appendCropQVector(&gp2[0], state_gp[0], gaitPhasePlotMaxNum2);
        appendCropQVector(&gp2[1], state_gp[1], gaitPhasePlotMaxNum2);

        ui->plot_gaitPhase->xAxis->setRange(t_gp[0], t_gp[0] + 7.0);
        ui->plot_gaitPhase_2->xAxis->setRange(t_gp2[0], t_gp2[0] + 27.0);
        this->updatePlot(GAIT_PHASE);
    }

    appendCropQVector(&t_v_l, data[0], voltPlotMaxNum);
    for (int i = 0; i < 6; i++){
        if((i==0) || (i==2) || (i==4))  appendCropQVector(&v_l[i], -1.0, voltPlotMaxNum);
        else   appendCropQVector(&v_l[i], data[i+1], voltPlotMaxNum);
    }
    ui->plot_sole_left_voltage->xAxis->setRange(t_v_l[0], t_v_l[0]+5.0);

//    if (is_plot_data)   this->updateSolePlot(SOLE_LEFT, data);

    if (!is_plot_sole) return;
    this->updatePlot(SOLE_LEFT);

    if(is_left_calib_on){
        double t_now = ros::Time::now().toSec();
        if (t_now - t_left_calib > 2.0){
            ui->button_sole_calibration_left->setText("Left Sole\nCalibration");
            ui->button_sole_calibration_left->setStyleSheet("background-color: rgb(211, 211, 211)");
            is_left_calib_on = false;
            qnode.loadSoleZero(SOLE_LEFT);
        }
    }
}

void MainWindow::plotSoleRight(){
    float* data = qnode.getSoleRightData();
    appendCropQVector(&t_v_r, data[0], voltPlotMaxNum);
    for (int i = 0; i < 6; i++){
        if((i==0) || (i==2) || (i==4))  appendCropQVector(&v_r[i], -1, voltPlotMaxNum);
        else   appendCropQVector(&v_r[i], data[i+1], voltPlotMaxNum);
    }
    ui->plot_sole_right_voltage->xAxis->setRange(t_v_r[0], t_v_r[0]+5.0);

//    if (is_plot_data)   this->updateSolePlot(SOLE_RIGHT, data);

    if (!is_plot_sole) return;
    this->updatePlot(SOLE_RIGHT);
    
    if(is_right_calib_on){
        double t_now = ros::Time::now().toSec();
        if (t_now - t_right_calib > 2.0){
            ui->button_sole_calibration_right->setText("Right Sole\nCalibration");
            ui->button_sole_calibration_right->setStyleSheet("background-color: rgb(211, 211, 211)");
            is_right_calib_on = false;
            qnode.loadSoleZero(SOLE_RIGHT);
        }
    }
}

void MainWindow::plotKinematics(){
    if(!is_plot_data) return;
    qnode.getLink(linkX, linkY, linkZ);
    for(int i = 0; i < 7 ; i++){
        link[i]->start->setCoords(linkZ[i], linkY[i]);
        link[i]->end->setCoords(linkZ[i+1], linkY[i+1]);
    }
    ui->plot_kinematics->replot();
}

void MainWindow::plotPlantar(){
    if(!is_plot_data) return;
    float* data = qnode.getPlantarData();

    appendCropQVector(&t_m_p, data[0], motorPlotMaxNum);
    appendCropQVector(&m_p[0], abs(data[1]), motorPlotMaxNum);
    appendCropQVector(&m_p[1], abs(data[2]), motorPlotMaxNum);
    ui->plot_plantar_command->xAxis->setRange(t_m_p[0], t_m_p[0]+5.0);
    ui->plot_plantar_command->yAxis->setRange(0, max_torque[0] + 0.1);
    this->updatePlot(MOTOR_PLANTAR);
}

void MainWindow::plotDorsi(){
    if (!is_plot_data) return;
    float* data = qnode.getDorsiData();

    appendCropQVector(&t_m_d, data[0], motorPlotMaxNum);
    appendCropQVector(&m_d[0], abs(data[1]), motorPlotMaxNum);
    appendCropQVector(&m_d[1], abs(data[2]), motorPlotMaxNum);
    appendCropQVector(&m_d[2], data[3], motorPlotMaxNum);
    appendCropQVector(&m_d[3], data[4], motorPlotMaxNum);
    ui->plot_dorsi_command->xAxis->setRange(t_m_d[0], t_m_d[0]+5.0);
    ui->plot_dorsi_command->yAxis->setRange(0, max_torque[1] + 0.1);
    this->updatePlot(MOTOR_DORSI);
}

void MainWindow::plotTLA(){
    if (!is_plot_woc) return;
    float* data = qnode.getTLAData();

    appendCropQVector(&t_TLA, data[0], tlaPlotMaxNum);
    appendCropQVector(&tla[0], data[1], tlaPlotMaxNum);
//    appendCropQVector(&tla[1], data[2], tlaPlotMaxNum);
    ui->plot_TLA->xAxis->setRange(t_TLA[0], t_TLA[0] + 5.0);
    ui->plot_TLA->yAxis->setRange(-1, 1);
    this->updatePlot(TLA);
}

void MainWindow::plotWOC(){
    if (!is_plot_woc) return;
    WOCData data;
    qnode.getWOCData(data);

    woc_left[0].clear();
    woc_left[1].clear();
    woc_right[0].clear();
    woc_right[1].clear();
    for (int i = 0; i < 101; i++){
        std::cout << data[0][i] << ", " << data[1][i] << ", " << data[2][i] << ", " << data[3][i] << std::endl;
        woc_left[0].append(data[0][i]);
        woc_left[1].append(data[1][i]);
        woc_right[0].append(data[2][i]);
        woc_right[1].append(data[3][i]);
    }
    this->updatePlot(WOCD);
}

void MainWindow::updateGaitPhaseState(){
    if(!is_plot_data) return;
    float* data = qnode.getGaitPhase();
    QPen pen[2];
    pen[0].setColor(QColor(0,0,0));
    pen[1].setColor(QColor(255, 0, 0));

    state_gp[0] = data[1] - 1;
    state_gp[1] = data[2] - 1;
 
    if(state_gp[current_affected_side == 0] == 0){
        infLineThreshold[0]->setPen(pen[0]);
        infLineThreshold[1]->setPen(pen[1]);
    }
    else{
        infLineThreshold[0]->setPen(pen[1]);
        infLineThreshold[1]->setPen(pen[0]);
    }
    
    if(state_gp[current_affected_side == 1] == 0){
        infLineThreshold[2]->setPen(pen[0]);
        infLineThreshold[3]->setPen(pen[1]);
    }
    else{
        infLineThreshold[2]->setPen(pen[1]);
        infLineThreshold[3]->setPen(pen[0]);
    }
    ui->plot_sole_left_voltage->replot();
    ui->plot_sole_right_voltage->replot();
}

void MainWindow::dorsiZeroingDone(){
    ui->button_dorsi_zeroing_done->setStyleSheet("background-color: rgb(0, 200, 0)");
    updateLog("Dorsi Zeroing Done");
}

void MainWindow::initPlot(){ 
    QPen pen[6];
    pen[0].setColor(QColor(0,0,0));
    pen[1].setColor(QColor(255, 0, 0));
    pen[2].setColor(QColor(0, 0, 255));
    pen[3].setColor(QColor(0, 255, 0));
    pen[4].setColor(QColor(255, 165, 0));
    pen[5].setColor(QColor(198, 115, 255));

    // Initialize kinematics plot
    for (int i = 0; i < 7; i++){
        link[i] = new QCPItemLine(ui->plot_kinematics);
        if (i > 4) link[i]->setPen(pen[1]);
        else link[i]->setPen(pen[0]);
    }
    linkX = new double[8];
    linkY = new double[8];
    linkZ = new double[8];

    ui->plot_kinematics->xAxis->setTicks(false);
    ui->plot_kinematics->yAxis->setTicks(false);
    ui->plot_kinematics->xAxis2->setVisible(true);
    ui->plot_kinematics->yAxis2->setVisible(true);
    ui->plot_kinematics->xAxis2->setTicks(false);
    ui->plot_kinematics->yAxis2->setTicks(false);

    ui->plot_kinematics->xAxis->setRange(-0.8 * 31.5/22.0, 0.8 * 31.5/22.0);
    ui->plot_kinematics->yAxis->setRange(-0.1, 1.5);

    state_gp[0] = 0.0;
    state_gp[1] = 0.0;

    // Initialize sole sensor plot
    for(int i = 0; i< 6; i++){
        ui->plot_sole_left_voltage->addGraph();
        ui->plot_sole_right_voltage->addGraph();
        ui->plot_sole_left_voltage->graph(i)->setPen(pen[i]);
        ui->plot_sole_right_voltage->graph(i)->setPen(pen[i]);
        ui->plot_sole_left_voltage->graph(i)->setName(QString(char(i)+'1'));
        ui->plot_sole_right_voltage->graph(i)->setName(QString(char(i)+'1'));
        
    }
    ui->plot_sole_left_voltage->yAxis->setRange(-0.05, 25);
    ui->plot_sole_right_voltage->yAxis->setRange(-0.05, 25);

    // Initialize controller plot
    for (int i = 0; i< 4; i++){
        ui->plot_dorsi_command->addGraph();
        ui->plot_dorsi_command->graph(i)->setPen(pen[i]);   
    }
    ui->plot_dorsi_command->graph(0)->setName(QString("t_d"));
    ui->plot_dorsi_command->graph(1)->setName(QString("t_a"));
    ui->plot_dorsi_command->graph(2)->setName(QString("p_d"));
    ui->plot_dorsi_command->graph(3)->setName(QString("p_a"));
    ui->plot_dorsi_command->yAxis->setRange(-1.2, 1.2);

    ui->plot_plantar_command->addGraph();
    ui->plot_plantar_command->addGraph();
    ui->plot_plantar_command->graph(0)->setPen(pen[0]);
    ui->plot_plantar_command->graph(1)->setPen(pen[1]);
    ui->plot_plantar_command->graph(0)->setName(QString("t_d"));
    ui->plot_plantar_command->graph(1)->setName(QString("t_a"));

    ui->plot_plantar_command->yAxis->setRange(-1.2, 1.2);

    // Initialize gait phase plot
    ui->plot_gaitPhase->addGraph();
    ui->plot_gaitPhase->addGraph();
    ui->plot_gaitPhase->graph(0)->setPen(pen[0]);
    ui->plot_gaitPhase->graph(1)->setPen(pen[1]);
    ui->plot_gaitPhase->graph(0)->setName("NP");
    ui->plot_gaitPhase->graph(1)->setName("P");
    ui->plot_gaitPhase->yAxis->setRange(-0.1, 1.2);

    ui->plot_gaitPhase_2->addGraph();
    ui->plot_gaitPhase_2->addGraph();
    ui->plot_gaitPhase_2->graph(0)->setPen(pen[0]);
    ui->plot_gaitPhase_2->graph(1)->setPen(pen[1]);
    ui->plot_gaitPhase_2->graph(0)->setName("NP");
    ui->plot_gaitPhase_2->graph(1)->setName("P");
    ui->plot_gaitPhase_2->yAxis->setRange(-0.1, 1.2);

    // Initialize woc plot
    ui->plot_TLA->addGraph();
    ui->plot_TLA->addGraph();
    ui->plot_TLA->graph(0)->setPen(pen[0]);
    ui->plot_TLA->graph(1)->setPen(pen[1]);
    ui->plot_TLA->graph(0)->setName("Left");
    ui->plot_TLA->graph(1)->setName("Right");

    ui->plot_optimized_control_left->addGraph();
    ui->plot_optimized_control_left->addGraph();
    ui->plot_optimized_control_left->graph(0)->setPen(pen[0]);
    ui->plot_optimized_control_left->graph(1)->setPen(pen[1]);
    ui->plot_optimized_control_left->graph(0)->setName("TLA");
    ui->plot_optimized_control_left->graph(1)->setName("WOC");

    ui->plot_optimized_control_right->addGraph();
    ui->plot_optimized_control_right->addGraph();
    ui->plot_optimized_control_right->graph(0)->setPen(pen[0]);
    ui->plot_optimized_control_right->graph(1)->setPen(pen[1]);
    ui->plot_optimized_control_right->graph(0)->setName("TLA");
    ui->plot_optimized_control_right->graph(1)->setName("WOC");

    for (int i = 0; i < 101; i++){
        t_woc.push_back(static_cast<float>(i) * 0.01);
    }

    // Set plot title
    ui->plot_sole_left_voltage->plotLayout()->insertRow(0);
    ui->plot_sole_right_voltage->plotLayout()->insertRow(0);
    ui->plot_plantar_command->plotLayout()->insertRow(0);
    ui->plot_dorsi_command->plotLayout()->insertRow(0);
    ui->plot_gaitPhase->plotLayout()->insertRow(0);
    ui->plot_gaitPhase_2->plotLayout()->insertRow(0);
    ui->plot_TLA->plotLayout()->insertRow(0);
    ui->plot_optimized_control_left->plotLayout()->insertRow(0);
    ui->plot_optimized_control_right->plotLayout()->insertRow(0);
    
    QCPTextElement *title_sole_left_voltage = new QCPTextElement(ui->plot_sole_left_voltage, "L Sole", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_sole_right_voltage = new QCPTextElement(ui->plot_sole_right_voltage, "R Sole", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_plantar_command = new QCPTextElement(ui->plot_plantar_command, "Plnatar", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_dorsi_command = new QCPTextElement(ui->plot_dorsi_command, "Dorsi", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_gaitPhase = new QCPTextElement(ui->plot_gaitPhase, "Gait Phase", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_gaitPhase2 = new QCPTextElement(ui->plot_gaitPhase_2, "Gait Phase", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_TLA = new QCPTextElement(ui->plot_TLA, "TLA", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_optimized_control_left = new QCPTextElement(ui->plot_optimized_control_left, "optimizer left", QFont("sans", 16, QFont::Bold));
    QCPTextElement *title_optimized_control_right = new QCPTextElement(ui->plot_optimized_control_right, "optimizer right", QFont("sans", 16, QFont::Bold));
    
    for (int i = 0 ; i < 2; i++){
        infLineThreshold[i] = new QCPItemStraightLine(ui->plot_sole_left_voltage);
        infLineThreshold[i+2] = new QCPItemStraightLine(ui->plot_sole_right_voltage);
    }
    updatePlotThreshold();

    ui->plot_sole_left_voltage->plotLayout()->addElement(0, 0, title_sole_left_voltage);
    ui->plot_sole_right_voltage->plotLayout()->addElement(0, 0, title_sole_right_voltage);
    ui->plot_plantar_command->plotLayout()->addElement(0, 0, title_plantar_command);
    ui->plot_dorsi_command->plotLayout()->addElement(0, 0, title_dorsi_command);
    ui->plot_gaitPhase->plotLayout()->addElement(0, 0, title_gaitPhase);
    ui->plot_gaitPhase_2->plotLayout()->addElement(0, 0, title_gaitPhase2);
    ui->plot_TLA->plotLayout()->addElement(0, 0, title_TLA);
    ui->plot_optimized_control_left->plotLayout()->addElement(0, 0, title_optimized_control_left);
    ui->plot_optimized_control_right->plotLayout()->addElement(0, 0, title_optimized_control_right);

    ui->plot_sole_left_voltage->legend->setVisible(true);
    ui->plot_sole_right_voltage->legend->setVisible(true);
    ui->plot_plantar_command->legend->setVisible(true);
    ui->plot_dorsi_command->legend->setVisible(true);
    ui->plot_gaitPhase->legend->setVisible(true);
    ui->plot_gaitPhase_2->legend->setVisible(true);
    ui->plot_sole_left_voltage->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop);
    ui->plot_sole_right_voltage->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop);
    ui->plot_TLA->setVisible(true);
    ui->plot_optimized_control_left->setVisible(true);
    ui->plot_optimized_control_right->setVisible(true);

    ui->plot_optimized_control_left->xAxis->setRange(0, 1);
    ui->plot_optimized_control_left->yAxis->setRange(-1.1, 1.1);
    ui->plot_optimized_control_right->xAxis->setRange(0, 1);
    ui->plot_optimized_control_right->yAxis->setRange(-1.1, 1.1);

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
        ui->plot_gaitPhase_2->graph(0)->setData(t_gp2, gp2[0]);
        ui->plot_gaitPhase_2->graph(1)->setData(t_gp2, gp2[1]);
        ui->plot_gaitPhase->replot();
        ui->plot_gaitPhase_2->replot();
    }

    else if (dataType == TLA){
        ui->plot_TLA->graph(0)->setData(t_TLA, tla[0]);
//        ui->plot_TLA->graph(1)->setData(t_TLA, tla[1]);
        ui->plot_TLA->replot();
    }

    else if (dataType == WOCD){
        ui->plot_optimized_control_left->graph(0)->setData(t_woc, woc_left[0]);
        ui->plot_optimized_control_left->graph(1)->setData(t_woc, woc_left[1]);
        ui->plot_optimized_control_right->graph(0)->setData(t_woc, woc_right[0]);
        ui->plot_optimized_control_right->graph(1)->setData(t_woc, woc_right[1]);
        ui->plot_optimized_control_left->replot();
        ui->plot_optimized_control_right->replot();
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
    s_r[0]->setShape(260, 180, 15, 15);
    s_r[1]->setShape(220, 70, 15, 15);
    s_r[2]->setShape(225, 145, 15, 15);
    s_r[3]->setShape(180, 80, 15, 15);
    s_r[4]->setShape(180, 140, 15, 15);
    s_r[5]->setShape(230, 350, 15, 15);
    
    ui->label_sole_image->stackUnder(ui->widget);
}

void MainWindow::updateSolePlot(int side, float* data){
    if (side == SOLE_LEFT){
        for (int i = 0; i < 6; i++){
            updateRGB(data[i+1] * 3);
            s_l[i]->redraw(rgb[0], rgb[1], rgb[2]);
        }
    }
    else{
        for (int i = 0; i < 6; i++){
            updateRGB(data[i+1] * 3);
            s_r[i]->redraw(rgb[0], rgb[1], rgb[2]);
        }
    }
}

void MainWindow::updateRGB(float f){
    float a = (1-f)/0.25;
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

std::string CutOnDecimalPt(std::string num, int pos)
{
    return num.substr(0, num.find('.') + pos + 1);
}
