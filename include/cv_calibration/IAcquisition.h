#ifndef __IACQUISITION_H__
#define __IACQUISITION_H__

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include "IImageProcess.h"
#include <chrono>
#include <ctime>

using namespace std::chrono;
class IAcquisition : public IImageProcess
{
protected:
    system_clock::time_point m_start;
    std::chrono::duration<double> m_timeout = milliseconds(5000);

    bool timedOut()
    {
        auto now = system_clock::now();
        auto delta = now - m_start;
        if (delta > m_timeout)
        {
            ROS_WARN("Timed out");
            return true;
        }
        return false;
    }

    void resetTimeout()
    {
        m_start = system_clock::now();
    }
public:
    void setTimeout(int millis)
    {
        m_timeout = milliseconds(millis);
    }

    virtual bool setInput(cv::InputArray in) override
    {
        return false;
    }

    IAcquisition()
    {
        m_result = ImageProcessResult();
        m_result.EntryPoint = this;
    }

    virtual ~IAcquisition(){}
};


#endif // __IACQUISITION_H__