#ifndef POINTCLOUDVIEWER_CPPQTW_H
#define POINTCLOUDVIEWER_CPPQTW_H

#include <QWidget>
#include <QtCore>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPoint>
#include <QPen>
#include "Dense"
#include <math.h>
#include <algorithm>
#include <unistd.h>
#include <iostream>

using Eigen::MatrixXd;

class PointCloudViewer_CppQtW : public QWidget
{
    Q_OBJECT
private:
    QWidget *parent;
    float windowX;
    float windowY;
    bool mouseLeftIsPressed;
    int mouseLeftPressedPositionX;
    int mouseLeftPressedPositionY;
    bool mouseMiddlePressed;
    int mouseMiddlePressedPositionX;
    int mouseMiddlePressedPositionY;
    float zoomFactor;
    int mouseLastPositionX;
    int mouseLastPositionY;
    int pointCloudPointsNum;
    bool enableStreaming;
    Eigen::MatrixXf pointCloud;
    Eigen::Matrix3f rotationMatrix;
public:
    explicit PointCloudViewer_CppQtW(int pointCloudPointsNum = 10000,bool enableStreaming = true,QWidget *parent = nullptr);
    void RotateX(float dPfi);
    void RotateY(float dPhi);
    void RotateZ(float dPhi);
    void EnableStreaming(bool enable);
    void ResizePointCloud(int pointCloudPointsNum);
    void Refresh();
    void StreamIn(Eigen::MatrixXf *stream);
    void CentralizePointCloud();
protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

signals:

};

#endif // POINTCLOUDVIEWER_CPPQTW_H
