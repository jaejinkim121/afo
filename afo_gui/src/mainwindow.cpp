
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc, argv)
    , ui(new Ui::MainWindow)
{
    qnode.init();
    ui->setupUi(this);
    
    // Connect QObject to ui objects.
    QObject::connect(ui->test, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(ui->button_toggle_plot, SIGNAL(clicked()), this, SLOT(buttonClicked()));
    QObject::connect(&qnode, SIGNAL(updateSoleLeft()), this, SLOT(plotSoleLeft()));
    QObject::connect(&qnode, SIGNAL(updateSoleRight()), this, SLOT(plotSoleRight()));
    QObject::connect(&qnode, SIGNAL(updatePlantar()), this, SLOT(plotPlantar()));
    QObject::connect(&qnode, SIGNAL(updateDorsi()), this, SLOT(plotDorsi()));

    this->initPlot();
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
        this->togglePlot();
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

void MainWindow::togglePlot(){
    is_plot = !is_plot;
    if (!is_plot){
        t_v_l.clear();
        t_v_r.clear();
        t_m_p.clear();
        t_m_d.clear();
        for (int i = 0; i < 6; i++){
            v_l[i].clear();
            v_r[i].clear();
        }
        for (int i =0; i< 2; i++){
            m_p[i].clear();
        }
        for (int i = 0; i< 4; i++){
            m_d[i].clear();
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

void MainWindow::plotSoleLeft(){
    if (!is_plot) return;
    float* data = qnode.getSoleLeftData();
    appendCropQVector(&t_v_l, data[0], voltPlotMaxNum);
    for (int i = 0; i < 6; i++){
        appendCropQVector(&v_l[i], data[i], voltPlotMaxNum);
    }
    ui->plot_sole_left_voltage->xAxis->setRange(t_v_l[0], t_v_l[0]+5.0);
    this->updatePlot(SOLE_LEFT);
}

void MainWindow::plotSoleRight(){
    if (!is_plot) return;
    float* data = qnode.getSoleRightData();
    appendCropQVector(&t_v_r, data[0], voltPlotMaxNum);
    for (int i = 0; i < 6; i++){
        appendCropQVector(&v_r[i], data[i], voltPlotMaxNum);
    }
    ui->plot_sole_right_voltage->xAxis->setRange(t_v_r[0], t_v_r[0]+5.0);
    this->updatePlot(SOLE_RIGHT);
}

void MainWindow::plotPlantar(){
    if(!is_plot) return;
    float* data = qnode.getPlantarData();

    appendCropQVector(&t_m_p, data[0], motorPlotMaxNum);
    appendCropQVector(&m_p[0], data[1], motorPlotMaxNum);
    appendCropQVector(&m_p[1], data[2], motorPlotMaxNum);
    ui->plot_plantar_command->xAxis->setRange(t_m_p[0], t_m_p[0]+5.0);
    this->updatePlot(MOTOR_PLANTAR);
}

void MainWindow::plotDorsi(){
    if (!is_plot) return;
    float* data = qnode.getDorsiData();

    appendCropQVector(&t_m_d, data[0], motorPlotMaxNum);
    appendCropQVector(&m_d[0], data[1], motorPlotMaxNum);
    appendCropQVector(&m_d[1], data[2], motorPlotMaxNum);
    appendCropQVector(&m_d[2], data[3], motorPlotMaxNum);
    appendCropQVector(&m_d[3], data[4], motorPlotMaxNum);
    ui->plot_dorsi_command->xAxis->setRange(t_m_d[0], t_m_d[0]+5.0);
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
        ui->plot_sole_right_voltage->graph(i)->setPen(pen[i]);
        
    }
    ui->plot_sole_left_voltage->yAxis->setRange(0, 3.3);
    ui->plot_sole_right_voltage->yAxis->setRange(0, 3.3);

    for (int i = 0; i< 4; i++){
        ui->plot_dorsi_command->addGraph();
        ui->plot_dorsi_command->graph(i)->setPen(pen[i]);   
    }
    ui->plot_dorsi_command->yAxis->setRange(-1.2, 1.2);

    ui->plot_plantar_command->addGraph();
    ui->plot_plantar_command->addGraph();
    ui->plot_plantar_command->graph(0)->setPen(pen[0]);
    ui->plot_plantar_command->graph(1)->setPen(pen[1]);
    ui->plot_plantar_command->yAxis->setRange(-1.2, 1.2);
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


