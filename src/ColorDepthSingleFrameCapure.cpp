#include "ColorDepthSingleFrameCapure.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <ros/ros.h>
void ColorDepthSingleFrameCapture::run()
{
    if (!m_capture) return;
    m_capture = false;
    ROS_INFO("Open streams");


    

    // cv::Mat c;
    // cv::Mat d;
    // c.addref();
    // d.addref();

    std::vector<int> ready {0,0};
    std::vector<cv::VideoCapture> streams {m_colorStream, m_depthStream};

    if (!m_colorStream.isOpened()) m_colorStream.open(m_colorAddress);
    if (!m_depthStream.isOpened()) m_depthStream.open(m_depthAddress);
    if (!cv::VideoCapture::waitAny(streams, ready, 50)) return;

    if (ready[0])
    {
        ROS_INFO("Color opened");
        lock(m_colorLock)
        {
            if (m_colorStream.grab())
                ROS_INFO("Failed to read color");
        }
        //m_colorStream.release();
    }

    if (ready[1])
    {
        ROS_INFO("Depth opened");
        lock(m_depthLock)
        {
            if (!m_depthStream.grab())
                ROS_INFO("Failed to read depth");
        }
        //m_depthStream.release();
    }
    //ROS_INFO("Streams released");
    // c.release();
    // d.release();
}

ColorDepthSingleFrameCapture::SingleFrameCapture& ColorDepthSingleFrameCapture::getDepthCapture()
{
    cv::Mat m;
    if (m_depthStream.isOpened() && m_depthStream.retrieve(m))
    {
        m_depthCapture.setImage(m);
    }
    return m_depthCapture;
}

ColorDepthSingleFrameCapture::SingleFrameCapture& ColorDepthSingleFrameCapture::getColorCapture()
{
    cv::Mat m;
    if (m_colorStream.isOpened() && m_colorStream.retrieve(m))
    {
        m_colorCapture.setImage(m);
    }
    return m_colorCapture;
}

ColorDepthSingleFrameCapture::ColorDepthSingleFrameCapture(std::string colorAddress, std::string depthAddress)
{
    m_colorAddress = colorAddress;
    m_depthAddress = depthAddress;
}