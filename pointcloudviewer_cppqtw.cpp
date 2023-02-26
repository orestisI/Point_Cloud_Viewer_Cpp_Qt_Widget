#include "pointcloudviewer_cppqtw.h"

PointCloudViewer_CppQtW::PointCloudViewer_CppQtW(int pointCloudPointsNum,bool enableStreaming,QWidget *parent)
    : QWidget{parent}
{
    this->parent = parent;
    this->setMouseTracking(true);
    this->setAttribute(Qt::WA_StyledBackground ,true);
    this->setStyleSheet("background-color:black;");
    this->windowX = 0;
    this->windowY = 0;
    this->mouseLeftIsPressed = false;
    this->mouseLeftPressedPositionX = 0;
    this->mouseLeftPressedPositionY = 0;
    this->mouseMiddlePressed = false;
    this->mouseMiddlePressedPositionX = 0;
    this->mouseMiddlePressedPositionY = 0;
    this->zoomFactor = 64;
    this->mouseLastPositionX = 0;
    this->mouseLastPositionY = 0;
    this->pointCloudPointsNum = pointCloudPointsNum;
    this->enableStreaming = enableStreaming;
    this->pointCloud.resize(3,pointCloudPointsNum);
    this->pointCloud.setZero();
    this->rotationMatrix(0,0) = 1.0;
    this->rotationMatrix(0,1) = 0.0;
    this->rotationMatrix(0,2) = 0.0;
    this->rotationMatrix(1,0) = 0.0;
    this->rotationMatrix(1,1) = 1.0;
    this->rotationMatrix(1,2) = 0.0;
    this->rotationMatrix(2,0) = 0.0;
    this->rotationMatrix(2,1) = 0.0;
    this->rotationMatrix(2,2) = 1.0;
}

void PointCloudViewer_CppQtW::RotateX(float dPfi){
    Eigen::Matrix3f t;
    t(0,0) = 1.0;
    t(0,1) = 0.0;
    t(0,2) = 0.0;
    t(1,0) = 0.0;
    t(1,1) = cos(dPfi);
    t(1,2) = - sin(dPfi);
    t(2,0) = 0.0;
    t(2,1) = sin(dPfi);
    t(2,2) = cos(dPfi);
    this->rotationMatrix = t * this->rotationMatrix;
}

void PointCloudViewer_CppQtW::RotateY(float dPfi){
    Eigen::Matrix3f t;
    t(0,0) = cos(dPfi);
    t(0,1) = 0.0;
    t(0,2) = sin(dPfi);
    t(1,0) = 0.0;
    t(1,1) = 1.0;
    t(1,2) = 0.0;
    t(2,0) = - sin(dPfi);
    t(2,1) = 0.0;
    t(2,2) = cos(dPfi);
    this->rotationMatrix = t * this->rotationMatrix;
}

void PointCloudViewer_CppQtW::RotateZ(float dPfi){
    Eigen::Matrix3f t;
    t(0,0) = cos(dPfi);
    t(0,1) = - sin(dPfi);
    t(0,2) = 0.0;
    t(1,0) = sin(dPfi);
    t(1,1) = cos(dPfi);
    t(1,2) = 0.0;
    t(2,0) = 0.0;
    t(2,1) = 0.0;
    t(2,2) = 1.0;
    this->rotationMatrix = t * this->rotationMatrix;
}

void PointCloudViewer_CppQtW::EnableStreaming(bool enable){
    this->enableStreaming = enable;
}

void PointCloudViewer_CppQtW::ResizePointCloud(int pointCloudPointsNum){
    this->pointCloudPointsNum = pointCloudPointsNum;
    this->pointCloud.resize(3,this->pointCloudPointsNum);
    this->pointCloud.setZero();
}

void PointCloudViewer_CppQtW::Refresh(){
    this->pointCloud.setZero();
    this->update();
}

void PointCloudViewer_CppQtW::StreamIn(Eigen::MatrixXf *stream){
   if (this->enableStreaming){
        int cols =(int)stream->cols();
        int n = std::min(cols,this->pointCloudPointsNum);
        for (int i=0; i<this->pointCloudPointsNum; i++){
            this->pointCloud(0,i) = this->pointCloud(0,(i + n)%this->pointCloudPointsNum);
            this->pointCloud(1,i) = this->pointCloud(1,(i + n)%this->pointCloudPointsNum);
            this->pointCloud(2,i) = this->pointCloud(2,(i + n)%this->pointCloudPointsNum);
        }
        for (int i=1; i<n; i++){
            this->pointCloud(0,this->pointCloudPointsNum - i) = stream->coeff(0,i);
            this->pointCloud(1,this->pointCloudPointsNum - i) = stream->coeff(1,i);
            this->pointCloud(2,this->pointCloudPointsNum - i) = stream->coeff(2,i);
        }
        this->CentralizePointCloud();
        this->update();
   }
}

void PointCloudViewer_CppQtW::paintEvent(QPaintEvent *event){
    QPainter painter(this);
    QPen pen = QPen(Qt::white,3,Qt::SolidLine);
    painter.setPen(pen);
    Eigen::MatrixXf projectionMatrix = this->rotationMatrix * this->pointCloud;
    for (int i=0; i<this->pointCloudPointsNum; i++){
        float x = (float) (projectionMatrix.coeff(0,i) - this->windowX)*this->zoomFactor;
        float y = (float) (projectionMatrix.coeff(1,i) - this->windowY)*this->zoomFactor;
        int xP = (int ) x;
        int yP = (int ) y;
        painter.drawPoint(xP,yP);
    }
    painter.end();
}

void PointCloudViewer_CppQtW::mousePressEvent(QMouseEvent *event){
    int x = event->x();
    int y = event->y();
    if (event->button() == Qt::LeftButton){
        this->mouseLeftIsPressed = true;
        this->mouseLeftPressedPositionX = x;
        this->mouseLeftPressedPositionY = y;
    }
    else if (event->button() == Qt::MiddleButton){
        this->mouseMiddlePressed = true;
        this->mouseMiddlePressedPositionX = x;
        this->mouseMiddlePressedPositionY = y;
    }
}

void PointCloudViewer_CppQtW::mouseReleaseEvent(QMouseEvent *event){
    if (event->button() == Qt::LeftButton){
        this->mouseLeftIsPressed = false;
    }
    else if (event->button() == Qt::MiddleButton){
        this->mouseMiddlePressed = false;
    }
}

void PointCloudViewer_CppQtW::mouseMoveEvent(QMouseEvent *event){
    int x = event->x();
    int y = event->y();
    this->mouseLastPositionX = x;
    this->mouseLastPositionY = y;
    if (this->mouseMiddlePressed){
        float dx =(float) (x - this->mouseMiddlePressedPositionX);
        float dy =(float) (y - this->mouseMiddlePressedPositionY);

        this->windowX = (float) (this->windowX - dx/this->zoomFactor);
        this->windowY = (float) (this->windowY - dy/this->zoomFactor);
        this->mouseMiddlePressedPositionX = x;
        this->mouseMiddlePressedPositionY = y;
    }
    else{
        if (this->mouseLeftIsPressed){
            float dPfi =(float) (x - this->mouseLeftPressedPositionX)/this->width() * 2 * M_PI;
            float dTheta =(float) (y - this->mouseLeftPressedPositionY)/this->height() * 2 * M_PI;
            this->RotateX(-dTheta);
            this->RotateY(dPfi);
            this->mouseLeftPressedPositionX = x;
            this->mouseLeftPressedPositionY = y;

        }
    }
    this->update();
}

void PointCloudViewer_CppQtW::wheelEvent(QWheelEvent *event){
    QPoint delta = event->angleDelta();
    if (delta.y() > 0){
        this->zoomFactor = this->zoomFactor * 2.0;
        this->windowX = this->windowX + (1.0 / this->zoomFactor)*this->mouseLastPositionX;
        this->windowY = this->windowY + (1.0 / this->zoomFactor)*this->mouseLastPositionY;
    }
    else{
        this->zoomFactor = this->zoomFactor / 2.0;
        this->windowX = this->windowX - (0.5 / this->zoomFactor)*this->mouseLastPositionX;
        this->windowY = this->windowY - (0.5 / this->zoomFactor)*this->mouseLastPositionY;
    }
    this->update();
}

void PointCloudViewer_CppQtW::CentralizePointCloud(){
     Eigen::VectorXf  m = this->pointCloud.rowwise().mean();

     Eigen::MatrixXf t(3,this->pointCloudPointsNum);
     for (int i=0; i<this->pointCloudPointsNum; i++){
         this->pointCloud(0,i) = this->pointCloud(0,i) - m(0);
         this->pointCloud(1,i) = this->pointCloud(1,i) - m(1);
         this->pointCloud(2,i) = this->pointCloud(2,i) - m(2);
     }
}
