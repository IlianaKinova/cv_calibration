#include "RGBEdgeDrawerProcess.h"

void RGBEdgeDrawerProcess::run()
{
    lock(m_lock)
    {
        if (m_bDrawOver)
            m_input.copyTo(m_output);
        else
            m_output = cv::Mat::zeros(m_input.size(), CV_8UC3);

        m_payload.Contours.clear();
        m_payload.Hierarchy.clear();
        for( size_t i = 0; i< m_payload.Contours.size(); i++ )
        {
            cv::Scalar color = cv::Scalar( m_rng.uniform(0, 256), m_rng.uniform(0,256), m_rng.uniform(0,256) );
            try
            {
                drawContours(m_output, m_payload.Contours, (int)i, color, 3, cv::LINE_8, m_payload.Hierarchy, 1 );
            }
            catch(const cv::Exception& e)
            {
                ROS_ERROR("%s", e.what());

            }
            
        }
        m_bIsReady = true;
    }
}

RGBEdgeDrawerProcess::RGBEdgeDrawerProcess(bool drawOver)
{
    m_bDrawOver = drawOver;
}

bool RGBEdgeDrawerProcess::process(cv::OutputArray out)
{
    while (!m_bIsReady);
    lock(m_lock)
    {
        m_bIsReady = false;
    }
    return true;
}
