#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/video/tracking.hpp"

struct Ball{
    float radius;
    cv::Mat pos;
    bool isValid = true;

    cv::Mat getPos(){
        return pos.clone();
    }

    Ball(cv::FileNode ball){
        pos = (cv::Mat_<float>(4,1) << float(ball["x"]), float(ball["y"]), 0, 1);
        radius = ball["r"];
        if(radius < 0)
            isValid = false;
    }
};

struct Camera{
    cv::Mat transform;
    float focalLengthX;
    float aspectRatio;

    cv::Mat readMatrix(const cv::FileNode &fn){
        cv::Mat m(4,4, CV_32F);

        for(int i = 0; i < 4; i++)
            for(int j = 0; j < 4; j++){
                m.at<float>(i,j) = fn[i * 4 + j];
            }

        return m;
    }

    cv::Mat getPosition(){
        return transform.col(3);
    }

    cv::Mat getViewingDirection(){
        cv::Mat ez = (cv::Mat_<float>(4,1) << 0, 0, 1, 0);
        return transform * ez;
    }

    Camera(cv::FileNode fn){
        transform = readMatrix(fn["transform"]);
        focalLengthX = fn["focalLengthX"];
        float w = fn["viewRect"]["width"];
        float h = fn["viewRect"]["height"];
        aspectRatio = 640.f/540.f;
    }
};

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    std::vector<Camera> cameras;
    std::vector<cv::Mat> realBallPositions;
    std::vector<std::vector<cv::Mat>> positionsOut;
    std::vector<float> timestampsOut;
    std::vector<std::vector<Ball>> measuredPositions; // of ball
    std::vector<float> timestamps;
    unsigned int timestampsIdx = 0;
    std::vector<float> realTimestamps;
    unsigned int realTimestampsIdx = 0;

    unsigned int time_ms = 0;
    float measurementTimeOffset = -0.15; // seconds

    cv::Mat viewTransform;

    cv::KalmanFilter kalmanFilter;

protected:
    virtual void paintEvent(QPaintEvent *event);

    void drawCamera(QPainter& p, Camera & c, int i);

    cv::Mat calcBallPosition(Camera& camera, Ball &ball);

    cv::Mat calcCameraImgPointPosition(Camera& c, cv::Mat point2Dv4f);
};
#endif // MAINWINDOW_H
