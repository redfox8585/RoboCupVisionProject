#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include <thread>
#include <opencv2/core.hpp>    // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp> // Gaussian Blur
#include <opencv2/highgui.hpp> // OpenCV window I/O
#include <iostream>
#include <opencv2/videoio.hpp> // Video write

using namespace cv;
using namespace std;

Scalar BALL_DETECTION_COLOR = Scalar(255, 255, 0);
struct Ball
{
    float x, y, r;
};

struct ViewDimensions
{
    float x, y, width, height;
};

/* This function crops the current frame into six images
that represent the six different views from the simulation. */
std::vector<Mat> crop_image(Mat currentFrame, vector<ViewDimensions> dimensions)
{
    std::vector<Mat> views;

    float frameWidth = currentFrame.size().width;
    float frameHeight = currentFrame.size().height;

    for (int i = 0; i < 6; i++)
    {
        // the origin of dimensions in the bottom left corner but the origin of opencv images is in the top left corner
        float x = dimensions[i].x * frameWidth;
        float width = dimensions[i].width * frameWidth;
        float y = (1.f - dimensions[i].y - dimensions[i].height) * frameHeight;
        float height = dimensions[i].height * frameHeight;

        Mat ROI(currentFrame, Rect(x, y, width, height));

        views.push_back(ROI.clone());
    }
    return views;
}

/* This function analyzes a view to detect the ball and store the ball position in ballPositions*/
Ball analyze_view(Mat view, int mode)
{

    std::vector<cv::Vec3f> balls;

    Ball noBall = {
        0,  // x coordiante
        0,  // y coordiante
        -1, // radius
    };      // no ball present if radius negative

    // convert colorspace for better ball detection from BGR to HSV.
    Mat hsv_conversion;
    cvtColor(view, hsv_conversion, COLOR_BGR2HSV);

    //Scalar ball_color_hsv(0, 255, 255);

    Mat ballMask1,
        ballMask2;

    inRange(hsv_conversion, Scalar(0, 70, 70), Scalar(20, 255, 255), ballMask1);
    inRange(hsv_conversion, Scalar(160, 70, 70), Scalar(179, 255, 255), ballMask2);

    Mat threshold_image;
    addWeighted(ballMask1, 1.0, ballMask2, 1.0, 0.0, threshold_image);
    GaussianBlur(threshold_image, threshold_image, cv::Size(9, 9), 2, 2);

    if (mode == 1)
    {
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(threshold_image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); i++)
        {
            Point2f center;
            float radius;
            vector<Point> contours_poly;

            approxPolyDP(contours[i], contours_poly, 3, true);
            minEnclosingCircle(contours_poly, center, radius);
            Vec3f b(center.x, center.y, radius);
            balls.push_back(b);
        }
    }
    else
    {
        //Use the Hough transform to detect circles (ball) in the combined threshold image
        HoughCircles(threshold_image, balls, HOUGH_GRADIENT, 1, threshold_image.rows / 8, 100, 20, 1, 100);
    }

    if (balls.size() == 0)
        return noBall;

    // uncomment to show frame with ball detection
    // Mat view_with_detected_ball;
    // view.copyTo(view_with_detected_ball);
    // Point2f center(balls[0][0], balls[0][1]);
    // circle(view_with_detected_ball, center, balls[0][2], BALL_DETECTION_COLOR, 2);
    // imshow("view_with_detected_ball", view_with_detected_ball);
    // waitKey();

    Ball ballPosition = {
        balls[0][0] / float(view.cols) * 2.f - 1.f,
        (1.f - balls[0][1] / float(view.rows)) * 2.f - 1.f,
        balls[0][2] / float(view.cols)};

    return ballPosition;
}

/* This function takes the original frames from the input video and the ballPosition,
 to draw a circle around the detected ball. Then it writes the changed frame to the output video*/
void write_video_with_ball_detection(Ball &ballPosition, Mat view, VideoWriter &outputVideo)
{
    Mat frame_with_ball_detection;
    view.copyTo(frame_with_ball_detection);

    int radius = std::round(ballPosition.r);
    if (ballPosition.r > 0)
    {
        cv::Point center(std::round(ballPosition.x), std::round(ballPosition.y));
        cv::circle(frame_with_ball_detection, center, radius, BALL_DETECTION_COLOR, 3);
    }

    outputVideo.write(frame_with_ball_detection);
}

void visualize_progress_bar(float currentFrameNumber, float frameCount)
{

    int barWidth = 70;
    float progress = 0.0;

    progress = currentFrameNumber / frameCount;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i)

    {

        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
};

void writeBallPositions(vector<vector<Ball>> &ballPositions, vector<double> timestamps)
{
    ofstream out;
    out.open("ballPositions.json");
    out << "{ \"positions\" : [";

    for (unsigned int i = 0; i < ballPositions.size(); i++)
    {
        if (i > 0)
            out << ", ";
        out << "[";
        for (unsigned int k = 0; k < ballPositions[i].size(); k++)
        {
            if (k > 0)
                out << ", ";
            out << "{ \"x\": " << ballPositions[i][k].x << ", \"y\": " << ballPositions[i][k].y << ", \"r\": " << ballPositions[i][k].r << " }";
        }
        out << "]";
    }
    out << "], \"timestamps\": [";

    for (unsigned int i = 0; i < timestamps.size(); i++)
    {
        if (i != 0)
            out << ", ";
        out << timestamps[i];
    }

    out << "]}";

    out.close();
}

vector<vector<Ball>> readBallPositions(string fileName)
{
    vector<vector<Ball>> ballPositions;

    FileStorage fs(fileName, FileStorage::READ);
    cv::FileNode root = fs["positions"];
    ballPositions.resize(root.size());
    for (unsigned int i = 0; i < root.size(); i++)
    {
        cv::FileNode n = root[i];
        ballPositions[i].resize(n.size());
        for (unsigned int k = 0; k < n.size(); k++)
        {
            ballPositions[i][k].x = n[k]["x"];
            ballPositions[i][k].y = n[k]["y"];
            ballPositions[i][k].r = n[k]["r"];
        }
    }

    return ballPositions;
}

vector<ViewDimensions> readViewDimensions(FileStorage &fs)
{
    FileNode cameras = fs["cameras"];

    vector<ViewDimensions> out(cameras.size());

    for (unsigned int i = 0; i < cameras.size(); i++)
    {
        FileNode camera = cameras[i];
        FileNode viewRect = camera["viewRect"];

        out[i].x = viewRect["x"];
        out[i].y = viewRect["y"];
        out[i].width = viewRect["width"];
        out[i].height = viewRect["height"];
    }
    return out;
}

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        printf("Not all arguments were given.\n");
        return -1;
    }

    try
    {
        FileStorage fs(argv[2], 0);

        cv::VideoCapture inputVideo;

        inputVideo.open(argv[1]);
        unsigned int frameCount = inputVideo.get(CAP_PROP_FRAME_COUNT);
        Size S = Size((int)inputVideo.get(CAP_PROP_FRAME_WIDTH),
                      (int)inputVideo.get(CAP_PROP_FRAME_HEIGHT));
        int ex = static_cast<int>(inputVideo.get(CAP_PROP_FOURCC)); // Get Codec Type- Int form

        vector<ViewDimensions> dimensions = readViewDimensions(fs);
        vector<vector<Ball>> ballPositions;
        vector<double> timeStamps;

        if (argc > 4)
        {
            ballPositions = readBallPositions("ballPositions.json");
            for (unsigned int k = 0; k < ballPositions.size() && k < frameCount; k++)
            {
                cv::Mat frame;
                inputVideo >> frame;

                for (int i = 0; i < 6; i++)
                {

                    if (ballPositions[k][i].r > 0.f)
                    {
                        // the origin of dimensions is in the bottom left corner but the origin of opencv images is in the top left corner
                        float x = dimensions[i].x * S.width;
                        float w = dimensions[i].width * S.width;
                        float y = (1.f - dimensions[i].y) * S.height;
                        float h = dimensions[i].height * S.height;

                        int radius = std::round(ballPositions[k][i].r * w);

                        cv::Point center(std::round((ballPositions[k][i].x + 1.f) / 2.f * w + x), std::round(y - (ballPositions[k][i].y + 1.f) / 2.f * h));
                        cv::circle(frame, center, radius, BALL_DETECTION_COLOR, 3);
                    }
                }

                imshow("dectected positions", frame);
                cv::waitKey();
            }
        }
        else
        {
            //VideoWriter outputVideo("output.avi", ex, inputVideo.get(CAP_PROP_FPS), , true);

            if (!inputVideo.isOpened())
            {
                std::cout << "Error opening video stream or file" << std::endl;
                return -1;
            }

            ballPositions.resize(frameCount);
            timeStamps.resize(frameCount);
            std::vector<Mat> views;
            Mat currentFrame;

            for (unsigned int k = 0; k < frameCount; k++)
            {
                visualize_progress_bar(k, frameCount);
                inputVideo >> currentFrame;

                if (currentFrame.empty())
                    break;

                views = crop_image(currentFrame, dimensions);
                for (unsigned int i = 0; i < views.size(); i++)
                {
                    ballPositions[k].push_back(analyze_view(views[i], atoi(argv[3])));
                }

                timeStamps[k] = inputVideo.get(cv::CAP_PROP_POS_MSEC) / 1000.0; // write seconds
                //write_video_with_ball_detection(ballPositions[k][1], views[1], outputVideo);
            }
            writeBallPositions(ballPositions, timeStamps);

            //outputVideo.release();
        }
        inputVideo.release();
    }
    catch (cv::Exception &e)
    {
        printf("exception");
        std::cout << e.msg + "\n";
        exit(1);
    }

    return 0;
}
