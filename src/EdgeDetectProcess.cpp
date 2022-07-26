#include "EdgeDetectProcess.h"

void EdgeDetectProcess::run()
{
    cv::RNG m_rng{12345};
    
    lock(m_lock)
    {
        cv::Canny( m_input, m_canny_output, m_cannyTresh1, m_cannyTresh2 );
        cv::findContours(m_canny_output, m_payload.Contours, m_payload.Hierarchy, m_mode, m_approx);
        m_input.copyTo(m_output);
        m_bIsReady = true;
    }
}

bool EdgeDetectProcess::process(cv::OutputArray out)
{
    while (!m_bIsReady);
    lock(m_lock)
    {
        m_output.copyTo(out);
        m_bIsReady = false;
    }
    return true;
}
