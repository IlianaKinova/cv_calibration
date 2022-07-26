#include "DisplayProcess.h"
#include "opencv2/highgui.hpp"
#include <ros/ros.h>

void DisplayProcess::run()
{
    if (m_ready)
    {
        try
        {
            lock(m_lock)
            {
                cv::imshow(m_name, m_input);
            }
        }
        catch(cv::Exception)
        {
            ROS_ERROR("imshow fail");
        }
        
        m_ready = false;
    }
}

DisplayProcess::DisplayProcess(std::string name)
{
    m_name = name;
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
}

bool DisplayProcess::setInput(cv::InputArray in)
{
    lock(m_lock)
    {
        in.copyTo(m_input);
    }
    //m_ready = m_input.cols != 0 && m_input.rows != 0;
    m_ready = m_result.Done = m_result.Success;
    ROS_DEBUG("Passing to capture");
    passResultTo(*m_result.EntryPoint);
    return m_result.Success;
}

bool DisplayProcess::process(cv::OutputArray out)
{
    return false;
}