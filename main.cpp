
#include <stdio.h>
#include <vector>
#include <string>
#include <opencv2/core.hpp>    // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc.hpp> // Gaussian Blur
#include <opencv2/highgui.hpp> // OpenCV window I/O
#include <iostream>
#include <opencv2/videoio.hpp> // Video write

using namespace cv;

Scalar BALL_DETECTION_COLOR = Scalar(255, 255, 255);
struct Ball
{
    float x, y, r;
};

/* This function crops the current frame into six images
that represent the six different views from the simulation. */
std::vector<Mat> crop_image(Mat currentFrame, FileStorage fs)
{
    std::vector<Mat> views;

    struct view_dimensions
    {
        float x, y, width, height;
    };

    float frameWidth = currentFrame.size().width;
    float frameHeight = currentFrame.size().height;

    FileNode root = fs["cameras"];

    for (int i = 0; i < 6; i++)
    { // read json file to extract view position
        view_dimensions dimensions;
        Mat currentView;

        FileNode camera = root[i];
        FileNode viewRect = camera["viewRect"];

        dimensions.x = viewRect["x"];
        dimensions.y = viewRect["y"];
        dimensions.width = viewRect["width"];
        dimensions.height = viewRect["height"];

        // printf("View: \nx: %6.4lf\ny: %6.4lf\nwidth: %6.4lf\nheight: %6.4lf\n", view.x, view.y, view.width, view.height);

        Mat ROI(currentFrame, Rect(dimensions.x * frameWidth, dimensions.y * frameHeight, dimensions.width * frameWidth, dimensions.height * frameHeight));
        ROI.copyTo(currentView);

        views.push_back(currentView);

        // imshow("cropped", croppedImage);
        // waitKey();
    }
    return views;
}

/* This function analyzes a view to detect the ball and store the ball position in ballPositions*/
void analyze_view(Mat view, Ball &ballPosition)
{

    Ball noBall = {
        -1, // x coordiante
        -1, // y coordiante
        -1, // radius
    };

    // convert colorspace for better ball detection from BGR to HSV.
    Mat hsv_conversion;
    cvtColor(view, hsv_conversion, COLOR_BGR2HSV);

    //Scalar ball_color_hsv(0, 255, 255);

    Mat ballMask1,
        ballMask2;

    inRange(hsv_conversion, Scalar(0, 70, 50), Scalar(30, 255, 255), ballMask1);
    inRange(hsv_conversion, Scalar(150, 70, 50), Scalar(179, 255, 255), ballMask2);

    Mat threshold_image;
    addWeighted(ballMask1, 1.0, ballMask2, 1.0, 0.0, threshold_image);
    GaussianBlur(threshold_image, threshold_image, cv::Size(9, 9), 2, 2);

    // Use the Hough transform to detect circles (ball) in the combined threshold image
    std::vector<cv::Vec3f>
        balls;
    HoughCircles(threshold_image, balls, HOUGH_GRADIENT, 1, threshold_image.rows / 8, 100, 20, 1, 100);
    if (balls.size() == 0)
    {
        ballPosition = noBall;
        return;
    };

    Ball temp_ball = {
        balls[0][0],
        balls[0][1],
        balls[0][2]};
    ballPosition = temp_ball;

    return;
}

/* This function takes the original frames from the input video and the ballPosition,
 to draw a circle around the detected ball. Then it writes the changed frame to the output video*/
void write_video_with_ball_detection(Ball &ballPosition, Mat view, VideoWriter &outputVideo)
{
    Mat frame_with_ball_detection;
    view.copyTo(frame_with_ball_detection);

    int radius = std::round(ballPosition.r);
    if (ballPosition.x != -1)
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

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        printf("Not all arguments were given.\n");
        return -1;
    }

    try
    {
        FileStorage fs(argv[2], 0);
        cv::VideoCapture inputVideo;

        inputVideo.open(argv[1]);
        int frameCount = inputVideo.get(CAP_PROP_FRAME_COUNT);
        Size S = Size((int)inputVideo.get(CAP_PROP_FRAME_WIDTH) * 0.3332,
                      (int)inputVideo.get(CAP_PROP_FRAME_HEIGHT) * 0.5);
        int ex = static_cast<int>(inputVideo.get(CAP_PROP_FOURCC)); // Get Codec Type- Int form

        VideoWriter outputVideo("output.avi", ex, inputVideo.get(CAP_PROP_FPS), S, true);

        if (!inputVideo.isOpened())
        {
            std::cout << "Error opening video stream or file" << std::endl;
            return -1;
        }

        Ball ballPosition;
        std::vector<Mat> views;
        Mat currentFrame;
        float currentFrameNumber = 0.0;

        for (;;)
        {
            visualize_progress_bar(currentFrameNumber, frameCount);

            inputVideo >> currentFrame;
            currentFrameNumber++;

            if (currentFrame.empty())
            {
                break;
            }

            views = crop_image(currentFrame, fs);
            analyze_view(views[1], ballPosition);
            write_video_with_ball_detection(ballPosition, views[1], outputVideo);
        }

        std::cout << "\nFinished" << std::endl;
        inputVideo.release();
        outputVideo.release();
    }
    catch (cv::Exception &e)
    {
        printf("exception");
        std::cout << e.msg + "\n";
        exit(1);
    }

    return 0;
}
