#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setCentralWidget(&this->pointCloudViewer_CppQtW);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::RandomStreaming(){

    while(true){
        Eigen::MatrixXf m = Eigen::MatrixXf::Random(3,10);
        this->pointCloudViewer_CppQtW.StreamIn(&m);
        usleep(10000);
        QApplication::processEvents();
    }

}

