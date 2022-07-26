#ifndef __SIMPLEACQUISITION_H__
#define __SIMPLEACQUISITION_H__

#include "IAcquisition.h"
#include <opencv2/highgui.hpp>

class SimpleAcquisition : public IAcquisition
{
private:
    cv::VideoCapture m_capture;
    std::string m_address;
    volatile bool m_ready = false;
protected:
    virtual void run() override;
public:
    virtual bool process(cv::OutputArray out) override;
    SimpleAcquisition(std::string address);
};








#endif // __SIMPLEACQUISITION_H__