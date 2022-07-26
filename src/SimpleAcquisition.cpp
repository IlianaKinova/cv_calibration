#include "SimpleAcquisition.h"


bool SimpleAcquisition::process(cv::OutputArray out)
{
    while (!m_ready) ;
    lock(m_lock)
    {
        m_output.copyTo(out);
    }
    m_ready = false;
    return true;
}

void SimpleAcquisition::run()
{
    if (m_result.Done)
    {
        m_result.Done = false;
        resetTimeout();
    }
    else if (timedOut()) 
    {
        m_capture.release();
    }

    if (!m_capture.isOpened()) 
    {
        //Error
        for (size_t i = 0; i < 5; i++)
        {
            if (m_capture.open(m_address))
            {
                ROS_INFO("stream opened");
                goto pass; // Skip delay
            }
            else
            {
                m_capture.release();
                ROS_ERROR("stream failed to open retrying");
            }
        }
        sleep(5);
    }
    pass:

    try
    {
        if (!m_capture.read(m_input)) 
        {
            ROS_WARN("capture fail");
        }
        else
        {
            lock(m_lock)
            {
                m_output = m_input;
            }
            m_ready = true;
        }
    }
    catch(const cv::Exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    ROS_DEBUG("Acquired");
}


SimpleAcquisition::SimpleAcquisition(std::string address)
{
    m_address = address;
    m_capture.setExceptionMode(true);
}
