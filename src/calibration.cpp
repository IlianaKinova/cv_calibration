#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/shape.hpp>
#include <opencv2/video.hpp>
#include "ros/ros.h"
#include <thread>
#include "threads.h"
#include <atomic>
#include <mutex>
#include <vector>
#include "lock.h"
#include "RosParams.h"
#include "SimpleAcquisition.h"
#include "DisplayProcess.h"
#include "ProcessingChain.h"
#include "ColorDepthSingleFrameCapure.h"
//#include "EdgeDetectProcess.h"
//#include "RGBEdgeDrawerProcess.h"
//#include "my_pkg/RobotControl.h"

ros::NodeHandle* n;
cv::VideoCapture colorCapture;
cv::VideoCapture depthCapture;

volatile bool run = true;

cv::Mat lastColor;
cv::Mat lastDepth;
volatile std::atomic<bool> frameReady{false};
volatile std::atomic<bool> frameDepthReady{false};

volatile std::atomic<int> colorCaptureCount{0};
volatile std::atomic<int> depthCaptureCount{0};
volatile std::atomic<int> colorShowCount{0};
volatile std::atomic<int> depthShowCount{0};


RosParams rosparams;
std::mutex colorLock;
std::mutex depthLock;

SimpleAcquisition* colorAcq;
SimpleAcquisition* depthAcq;
DisplayProcess* colorDisp;
DisplayProcess* depthDisp;

void captureColor()
{
    cv::Mat frame;
    static int frames = 0;
    while (run)
    {

        if (frames >= 300)
        {
            frames = 0;
            colorCapture.release();
        }

        if (!colorCapture.isOpened()) 
        {
            //Error
            for (size_t i = 0; i < 5; i++)
            {
                //if (colorCapture.open(rosparams.color_address))
                if (colorCapture.open(0))
                {
                    ROS_INFO("Color stream opened");
                    colorCapture.set(cv::CAP_PROP_BUFFERSIZE, 3);
                    break;
                }
                else
                {
                    colorCapture.release();
                    ROS_ERROR("Color stream failed to open retrying");
                }
            }
            sleep(5);
        }
        

        
        
        if (!colorCapture.read(frame)) 
        {
            ROS_WARN("Color capture fail");
        }
        lock(colorLock)
        {
            frame.copyTo(lastColor);
            frameReady = true;
        }
        colorCaptureCount++;
        frames++;
    }
}

void processColor(cv::InputOutputArray inout, std::vector<std::vector<cv::Point>> contours)
{
    cv::RNG rng(12345);
    cv::Mat canny_output;
    cv::Canny( inout, canny_output, 120, 120*2 );
    std::vector<cv::Vec4i> hierarchy;
    // std::vector<std::vector<cv::Point>> contours;

    //cv::Mat procImg;
    //in.convertTo(procImg, CV_32F, 1.0/255.0);
    cv::findContours(canny_output, contours, cv::RetrievalModes::RETR_TREE, cv::ContourApproximationModes::CHAIN_APPROX_TC89_KCOS);

    cv::Mat drawing = cv::Mat::zeros(inout.size(), CV_8UC3);
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( inout, contours, (int)i, color, 3, cv::LINE_8, hierarchy, 1 );
    }
}
    
cv::Mat stripBlack(cv::Mat input)
{
    cv::Mat input_bgra;
    cv::cvtColor(input, input_bgra, cv::ColorConversionCodes::COLOR_BayerBG2BGRA);

    // find all white pixel and set alpha value to zero:
    for (int y = 0; y < input_bgra.rows; ++y)
    for (int x = 0; x < input_bgra.cols; ++x)
    {
        cv::Vec4b & pixel = input_bgra.at<cv::Vec4b>(y, x);
        // if pixel is white
        if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0)
        {
            // set alpha to zero:
            pixel[3] = 0;
        }
    }
    return input_bgra;
}

void colorShapeProcess(std::vector<std::vector<cv::Point>> contours, cv::InputOutputArray inout)
{
    cv::Mat approx;
    for (int i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(contours, approx, 0.01 * cv::arcLength(contours, true), true);
        if (approx.elemSize() == 4)
        {
            drawContours( inout, contours, i, {0,255,0});
        }
        else
        {
            drawContours( inout, contours, i, {255,0,0});
        }
    }
    
}

void showColor()
{
    int k = -1;
    while (run)
    {
        while (run && !frameReady) 
        {
            k = cv::waitKey(1);
            if (k != -1) goto end;
        }

        cv::Mat proc;
        cv::Mat procContour;
        cv::Mat orig;
        std::vector<std::vector<cv::Point>> contours;
        lock (colorLock)
        {
            lastColor.copyTo(orig);
            frameReady = false;
        }
        orig.copyTo(proc);
        orig.copyTo(procContour);
        processColor(proc, contours);
        //colorShapeProcess(contours, procContour);
        
        cv::Mat res;
        cv::Mat res2;
        cv::addWeighted(orig, 0.4, proc, 0.2, 0, res);
        //cv::addWeighted(res, 0.4, procContour, 0.2, 0, res2);
        cv::imshow("Color", res);
        colorShowCount++;
    }
    end:
    run = false;
}

void captureDepth()
{
    cv::Mat frame;
    while (run)
    {
        if (!depthCapture.isOpened()) {
            //Error
            depthCapture.release();
            for (size_t i = 0; i < 5; i++)
            {
                if (depthCapture.open(rosparams.depth_address))
                {
                    ROS_INFO("Depth stream opened");
                    break;
                }
                else
                {
                    depthCapture.release();
                    ROS_ERROR("Depth stream failed to open retrying");
                }
            }
            sleep(5);
        }
        

        if (!depthCapture.read(frame)) {
            //Error
            ROS_WARN("Depth capture fail");
        }
        lock(depthLock)
        {
            lastDepth = frame;
            frameDepthReady = true;
        }
        depthCaptureCount++;
    }
}

cv::Mat processDepth(cv::Mat in)
{
    cv::Mat res;
    cv::RNG rng(12345);
    cv::Mat canny_output;
    cv::Canny( in, canny_output, 2, 2*2 );
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    //cv::Mat procImg;
    //in.convertTo(procImg, CV_32F, 1.0/255.0);
    cv::findContours(canny_output, contours, cv::RetrievalModes::RETR_TREE, cv::ContourApproximationModes::CHAIN_APPROX_TC89_KCOS);

    cv::Mat drawing = cv::Mat::zeros(in.size(), CV_8UC3);
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( in, contours, (int)i, color, 3, cv::LINE_8, hierarchy, 1 );
    }
    return in;
}

void showDepth()
{
    int k = -1;
    while (run)
    {
        while (run && !frameDepthReady) 
        {
            k = cv::waitKey(1);
            if (k != -1) goto end;
        }
        lock (depthLock)
        {
            cv::Mat proc = processDepth(lastDepth);
            cv::imshow("Depth", proc);
            frameDepthReady = false;
        }
        k = cv::waitKey(5);
        if (k != -1) goto end;
        depthShowCount++;
    }
    end:
    run = false;
}

void captureStreams()
{
    cv::Mat depth;
    cv::Mat color;
    ROS_INFO("Starting capture");
    while (run)
    {
        if (!depthCapture.isOpened()) {
            //Error
            depthCapture.release();
            for (size_t i = 0; i < 5; i++)
            {
                if (depthCapture.open(rosparams.depth_address))
                {
                    ROS_INFO("Depth stream opened");
                    break;
                }
                else
                {
                    depthCapture.release();
                    ROS_ERROR("Depth stream failed to open retrying");
                }
            }
            sleep(5);
        }

        if (!colorCapture.isOpened()) 
        {
            //Error
            for (size_t i = 0; i < 5; i++)
            {
                if (colorCapture.open(rosparams.color_address))
                {
                    ROS_INFO("Color stream opened");
                    break;
                }
                else
                {
                    colorCapture.release();
                    ROS_ERROR("Color stream failed to open retrying");
                }
            }
            sleep(5);
        }
        
        if (!depthCapture.grab()) {
            //Error
            ROS_WARN("Depth capture fail");
        }
        if (!colorCapture.grab()) 
        {
            ROS_WARN("Color capture fail");
        }
        lock(depthLock)
        {
            frameDepthReady = true;
        }
        lock(colorLock)
        {
            frameReady = true;
        }
        depthCaptureCount++;
        colorCaptureCount++;
    }
}

void reportStats()
{
    static ros::Time nextReport = ros::Time::now();
    auto now = ros::Time::now();
    if (nextReport <= now)
    {
        ROS_INFO("Run count | ColorCap: %6d | ColorShow: %6d | DepthCap: %6d | DepthShow: %6d |",
        colorCaptureCount.load(), colorShowCount.load(), depthCaptureCount.load(), depthShowCount.load());
        colorCaptureCount = colorShowCount = depthCaptureCount = depthShowCount = 0;
        nextReport = now + ros::Duration(1);
    }
}



void processFrames()
{
    ROS_INFO("Starting process");
    while (run)
    {
        if (frameDepthReady)
        {
            lock (depthLock)
            {
                if (depthCapture.retrieve(lastDepth))
                {
                    cv::Mat proc = processDepth(lastDepth);
                    cv::imshow("Depth", proc);
                    frameDepthReady = false;
                    depthShowCount++;
                }
            }
        }
        if (frameReady)
        {
            cv::Mat proc;
            lock (colorLock)
            {
                lastColor.copyTo(proc);
                frameReady = false;
            }
            if (colorCapture.retrieve(lastColor))
            {
                // processColor(lastColor);
                cv::Mat res;
                cv::addWeighted(lastColor, 0.4, proc, 0.2, 0, res);
                cv::imshow("Color", proc);
                colorShowCount++;
            }
        }
    }
}

//ColorDepthSingleFrameCapture* captures;
// EdgeDetectProcess* colorEdgeDetect;
// RGBEdgeDrawerProcess* colorEdgeDrawer;

// void renderPipeline()
// {
//     //captures->start();
//     colorAcq->start();
//     colorDisp->start();
//     depthDisp->start();

//     // IMultiResultProcess<EdgeDetectResult>::ToImageProcessConverter colorEdges;
//     // colorEdges > *colorEdgeDetect > *colorEdgeDrawer;
//     // colorEdges.start();

//     cv::Mat res;
//     while (run)
//     {
//         // captures->getColorCapture().process(res);
//         *colorAcq >> *colorDisp;

//         // captures->getDepthCapture() >> *depthDisp;


//     }
    
//     // colorEdges.stop();
//     depthDisp->stop();
//     colorDisp->stop();
//     colorAcq->stop();
//     //captures->stop();
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "CameraReader");
    ros::NodeHandle node;
    n = &node;

    if (rosparams.readArgs())
        ROS_INFO("All args were read");
    else
        ROS_INFO("Some params used default values");
    
    cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
    
    // Define processes
    // colorAcq = new SimpleAcquisition(rosparams.color_address);
    // depthAcq = new SimpleAcquisition(rosparams.depth_address);
    //captures = new ColorDepthSingleFrameCapture(rosparams.color_address, rosparams.depth_address);
    // colorDisp = new DisplayProcess("Color");
    // depthDisp = new DisplayProcess("Depth");
    //colorEdgeDetect = new EdgeDetectProcess(120, 120 / 2);
    //colorEdgeDrawer = new RGBEdgeDrawerProcess(false);

    // Build process chain
    // ProcessingChain colorChain = ProcessingChain() >> *colorAcq >> *colorDisp;
    // ProcessingChain depthChain = ProcessingChain() >> *depthAcq >> *depthDisp;

    // Start process chain
    // colorChain.start();
    // depthChain.start();

    //captures->capture();
    //auto pipeline = std::thread(renderPipeline);

    // auto capture = std::thread(captureStreams);
    // auto display = std::thread(processFrames);

    auto colorCap = std::thread(captureColor);
    auto colorDisp = std::thread(showColor);
    

    auto rate = ros::Rate{30};
    while (run)
    {
        ros::spinOnce();
        switch (cv::waitKey(1000/30))
        {
            case 'c': // capture
                //captures->capture();
                while (cv::waitKey(1000/30) == 'c');
            break;
            case -1: // do nothing
            break;
            default:
                run = false;
            break;
        }
    }
    ROS_INFO("Stopping execution");

    //pipeline.join();
    // colorChain.stop();
    // depthChain.stop();

}
