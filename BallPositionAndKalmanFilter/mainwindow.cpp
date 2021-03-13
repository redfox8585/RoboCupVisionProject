#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPainter>
#include <QTimer>

#include <fstream>

void writeBallPositions(std::vector<std::vector<cv::Mat>> &ballPositions, const std::vector<std::string>& keys, std::vector<float> timestamps){
    std::ofstream out;
    out.open ("ballPositionsOut.json", std::ofstream::trunc);

    out << "{";
    for(int k = 0; k < keys.size(); k++){
        if(k > 0)
            out << ", ";
        out << "\"" << keys[k] << "\" : [";

        for(unsigned int i = 0; i < ballPositions[k].size(); i++){
            if(i > 0)
                out << ", ";

            out << "{ \"x\": " << ballPositions[k][i].at<float>(0,0) << ", \"y\": " << ballPositions[k][i].at<float>(1,0) << "}";
        }
        out << "]";
    }

    out << ", \"timestamps\":[";
    for(int i = 0; i < timestamps.size(); i++)
    {
        if(i > 0)
            out << ", ";
        out << timestamps[i];
    }

    out << "]}";

    out.close();
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QTimer *timer = new QTimer(this);
    timer->setInterval(30);
    connect(timer, &QTimer::timeout, this, QOverload<>::of(&MainWindow::update));
    timer->start();

    cv::FileStorage fs("simulation.json", cv::FileStorage::READ);
    cv::FileNode c = fs["cameras"];
    if(c.type() == cv::FileNode::SEQ)
        for(unsigned int i = 0; i < c.size(); i++)
        {
            cameras.emplace_back(c[i]);
        }

    cv::FileNode p = fs["positions"];
    for(unsigned int i = 0; i < p.size(); i++){
        realBallPositions.push_back((cv::Mat_<float>(4, 1) << p[i]["x"], p[i]["y"], 0.1f, 1.f));
        realTimestamps.push_back(p[i]["t"]);
    }

    cv::FileStorage fb("ballPositions.json", cv::FileStorage::READ);
    cv::FileNode root = fb["positions"];
    measuredPositions.resize(root.size());
    for(unsigned int i = 0; i < root.size(); i++){
        cv::FileNode n = root[i];
        measuredPositions[i].reserve(n.size());
        for(unsigned int k = 0; k < n.size(); k++)
            measuredPositions[i].emplace_back(n[k]);

    }

    cv::FileNode ts = fb["timestamps"];
    for(unsigned int i = 0; i < ts.size(); i++)
        timestamps.push_back(ts[i]);

    // Draufsicht
    viewTransform = (cv::Mat_<float>(4, 4) << 100, 0, 0, 500, 0, -100, 0, 500, 0, 0, 100, 500, 0, 0, 0, 1);

    // von der Seite
    //viewTransform = (cv::Mat_<float>(4, 4) << 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1) * viewTransform;

    // durch eine Kamera
    int camNum = 2;
    cv::Mat A_inv;
    cv::invert(cameras[camNum].transform, A_inv);
    viewTransform *= A_inv;


    float dt = 0.033;

    kalmanFilter.init(4, 12);

    kalmanFilter.transitionMatrix = (cv::Mat_<float>(4,4) << 1, 0, dt, 0, 0, 1, 0, dt,0, 0, 1, 0, 0, 0, 0, 1);
    kalmanFilter.measurementMatrix = (cv::Mat_<float>(12, 4) << 1,0,0,0, 0,1,0,0, 1,0,0,0, 0,1,0,0, 1,0,0,0, 0,1,0,0, 1,0,0,0, 0,1,0,0, 1,0,0,0, 0,1,0,0, 1,0,0,0, 0,1,0,0);
    setIdentity(kalmanFilter.processNoiseCov, cv::Scalar::all(1e-5));
    setIdentity(kalmanFilter.measurementNoiseCov, cv::Scalar::all(1e-3));
    setIdentity(kalmanFilter.errorCovPost, cv::Scalar::all(1));

    positionsOut.resize(2);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    float a = 0.01;
    cv::Mat rotX = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, cos(a), -sin(a), 0, 0, sin(a), cos(a), 0, 0, 0, 0, 0);
    cv::Mat rotY = (cv::Mat_<float>(4, 4) << cos(a), 0, sin(a), 0, 0, 1, 0, 0, -sin(a), 0, cos(a), 0, 0, 0, 0, 0);

    // add rotation
    //viewTransform = rotX * rotY * viewTransform;


    while(timestamps[timestampsIdx] -measurementTimeOffset < time_ms / 1000.f && timestampsIdx + 1 < timestamps.size())
        timestampsIdx++;

    while(realTimestamps[realTimestampsIdx] < time_ms / 1000.f && realTimestampsIdx + 1 < realTimestamps.size())
        realTimestampsIdx++;

    timestampsOut.push_back(time_ms/1000.f);

    std::vector<Ball>& balls = measuredPositions[timestampsIdx];
    cv::Mat realPosition3D = realBallPositions[realTimestampsIdx];
    positionsOut[0].push_back(realPosition3D.clone());

    float radius1 = 5;
    QPainter painter(this);

    cv::Mat measurments = cv::Mat::zeros(12, 1, CV_32F);

    for(int i : {0, 1, 2,3 ,4 ,5}){

        drawCamera(painter, cameras[i], i);

        if(!balls[i].isValid){
            //kalmanFilter.measurementMatrix.at<float>(i * 2, 0) = 0;
            //kalmanFilter.measurementMatrix.at<float>(i * 2 + 1, 1) = 0;
            kalmanFilter.measurementNoiseCov.at<float>(i*2, i*2) = 1000;
            kalmanFilter.measurementNoiseCov.at<float>(i*2 + 1, i*2 + 1) = 1000;
            continue;
        }

        //kalmanFilter.measurementMatrix.at<float>(i * 2, 0) = 1.f;
        //kalmanFilter.measurementMatrix.at<float>(i * 2 + 1, 1) = 1.f;

        kalmanFilter.measurementNoiseCov.at<float>(i*2, i*2) = 1e-3;
        kalmanFilter.measurementNoiseCov.at<float>(i*2 + 1, i*2 + 1) = 1e-3;

        cv::Mat proj = viewTransform * calcCameraImgPointPosition(cameras[i], balls[i].getPos());

        QPointF projection(proj.at<float>(0,0), proj.at<float>(1,0));

        painter.setPen(Qt::red);

        painter.drawEllipse(projection, radius1, radius1);


        cv::Mat ballP = calcBallPosition(cameras[i], balls[i]);
        measurments.at<float>(i * 2, 0) = ballP.at<float>(0,0);
        measurments.at<float>(i * 2 + 1, 0) =  ballP.at<float>(1,0);

        ballP = viewTransform * ballP;

        QPointF ballP_(ballP.at<float>(0,0), ballP.at<float>(1,0));
        painter.setPen(Qt::green);
        painter.drawEllipse(ballP_, radius1, radius1);

        cv::Mat p1 = viewTransform * cameras[i].getPosition();
        QPointF center(p1.at<float>(0,0), p1.at<float>(1,0));

        painter.drawLine(center, ballP_);

        cv::Mat realballP = viewTransform * realPosition3D;
        QPointF realballP_(realballP.at<float>(0,0), realballP.at<float>(1,0));
        painter.setPen(Qt::gray);
        painter.drawEllipse(realballP_, radius1, radius1);

    }

    painter.setPen(Qt::black);
    for(int x = -5; x < 5; x++)
        for(int y = -4; y < 4; y++)
            for(int i = 0; i < 2; i++)
            {
                cv::Mat p = viewTransform * (cv::Mat_<float>(4, 1) << x, y, 0, 0);
                QPointF center(viewTransform.at<float>(0,3) + p.at<float>(0, 0), viewTransform.at<float>(1,3) + p.at<float>(1, 0));
                QPointF p2(viewTransform.at<float>(0,i), viewTransform.at<float>(1,i));
                painter.drawLine(center, center + p2);
            }

    kalmanFilter.predict();

    cv::Mat state = kalmanFilter.correct(measurments);
    positionsOut[1].push_back(state.clone());

    float ballRadius = 0.1;
    cv::Mat ballKFPos = (cv::Mat_<float>(4,1) << state.at<float>(0,0), state.at<float>(1,0), ballRadius, 1);

    //painter.drawText(QPointF(10,10), QString::number(measurments.at<float>(0,0)) + ", " +QString::number(measurments.at<float>(1,0)));
    //painter.drawText(QPointF(10,10), QString::number(ballKFPos.at<float>(0,0)) + ", " +QString::number(ballKFPos.at<float>(1,0)));

    cv::Mat ballKF = viewTransform * ballKFPos;

    QPointF ballKF_(ballKF.at<float>(0,0), ballKF.at<float>(1,0));
    painter.setPen(Qt::black);
    painter.drawEllipse(ballKF_, radius1, radius1);

    /*painter.setPen(Qt::red);
    for(cv::Point3f p : ballPositions)
        painter.drawEllipse(p.x, p.y, radius1, radius1);

    painter.setPen(Qt::green);
    painter.drawEllipse(realBallPosition.x, realBallPosition.y, radius1, radius1);*/

    time_ms += 30;
    if(time_ms > 40000){

        std::vector<std::string> s({"realBallPositions", "measuredBallPositions"});
        writeBallPositions(positionsOut, s, timestampsOut);

        // repeat every 40 seconds
        /*time_ms = 0;
        timestampsIdx = 0;
        realTimestampsIdx = 0;*/
    }
}

void MainWindow::drawCamera(QPainter &painter, Camera &c, int i)
{
    float radius1 = 5;
    cv::Mat p1 = viewTransform * c.getPosition();
    QPointF center(p1.at<float>(0,0), p1.at<float>(1,0));

    painter.setPen(Qt::blue);
    painter.drawEllipse(center, radius1, radius1);
    painter.drawText(center + QPointF(10,10), QString::number(i));

    float focalL = c.focalLengthX;
    cv::Mat p2 = p1 - viewTransform * c.getViewingDirection() * focalL;
    QPointF p2_(p2.at<float>(0,0), p2.at<float>(1,0));

    painter.drawLine(center, p2_);

    // draw image plane
    cv::Mat p00 = viewTransform * calcCameraImgPointPosition(c, (cv::Mat_<float>(4, 1) << -1, -1, 0, 1));
    cv::Mat p01 = viewTransform * calcCameraImgPointPosition(c, (cv::Mat_<float>(4, 1) << -1, 1, 0, 1));
    cv::Mat p10 = viewTransform * calcCameraImgPointPosition(c, (cv::Mat_<float>(4, 1) << 1, -1, 0, 1));
    cv::Mat p11 = viewTransform * calcCameraImgPointPosition(c, (cv::Mat_<float>(4, 1) << 1, 1, 0, 1));

    painter.drawLine(QPointF(p00.at<float>(0,0),p00.at<float>(1,0)), QPointF(p01.at<float>(0,0),p01.at<float>(1,0)));
    painter.drawLine(QPointF(p01.at<float>(0,0),p01.at<float>(1,0)), QPointF(p11.at<float>(0,0),p11.at<float>(1,0)));
    painter.drawLine(QPointF(p11.at<float>(0,0),p11.at<float>(1,0)), QPointF(p10.at<float>(0,0),p10.at<float>(1,0)));
    painter.drawLine(QPointF(p10.at<float>(0,0),p10.at<float>(1,0)), QPointF(p00.at<float>(0,0),p00.at<float>(1,0)));
}

cv::Mat MainWindow::calcBallPosition(Camera &camera, Ball &ball)
{
    cv::Mat camP = camera.getPosition();
    float focalL = camera.focalLengthX;

    // ball position on camera image plane
    cv::Mat imgPlanePos = ball.getPos();
    imgPlanePos.at<float>(1,0) /= camera.aspectRatio;
    imgPlanePos.at<float>(2,0) = -focalL;
    imgPlanePos = camera.transform * imgPlanePos;

    // LGS Schnittpunkt: Ebene ex * u + ex * v + (0,0,radius)  Gerade p1 + t * d
    float ballRadius = 0.1;
    cv::Mat fieldPlaneOrigin = (cv::Mat_<float>(3,1) << 0, 0, ballRadius);
    cv::Mat b = camP.rowRange(0,3) - fieldPlaneOrigin;

    // LGS p1 - (0,0,radius) =  ex * u + ex * v - t * d    ==> b = A * x   ges.: x = A^(-1) * b
    cv::Mat d = imgPlanePos - camP; // Richtung der Sichtgeraden
    cv::Mat A = cv::Mat::eye(3,3, CV_32F); // identity matrix (first two columns are x and y direction of plane)
    // set last column to direction of plane
    for(int k = 0; k < 3; k++)
        A.at<float>(k, 2) = -d.at<float>(k, 0);

    cv::Mat A_inv;
    cv::invert(A, A_inv);

    cv::Mat x = A_inv * b; // solution x = (ball.x, ball.y, Sichtgeradenlänge)   | ball.z = radius

    x.resize(4);
    x.at<float> (2,0) = ballRadius;
    x.at<float>(3,0) = 1;

    return x;
}

cv::Mat MainWindow::calcCameraImgPointPosition(Camera &c, cv::Mat point2Dv4f)
{
    float focalL = c.focalLengthX;
    cv::Mat proj = point2Dv4f.clone();
    proj.at<float>(1,0) /= c.aspectRatio;
    proj.at<float>(2,0) = -focalL;
    return c.transform * proj;
}

