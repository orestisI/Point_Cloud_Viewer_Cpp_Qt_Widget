#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "Dense"
#include "pointcloudviewer_cppqtw.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void RandomStreaming();
private:
    Ui::MainWindow *ui;
    PointCloudViewer_CppQtW pointCloudViewer_CppQtW;
};
#endif // MAINWINDOW_H
