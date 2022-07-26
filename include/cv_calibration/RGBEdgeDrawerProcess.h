#ifndef __RGBEDGEDRAWERPROCESS_H__
#define __RGBEDGEDRAWERPROCESS_H__

#include "EdgeDetectProcess.h"


class RGBEdgeDrawerProcess : public IMultiResultProcess<EdgeDetectResult>
{
private:
    cv::RNG m_rng{12345};
    bool m_bDrawOver;
    volatile bool m_bIsReady = false;
protected:
    virtual void run() override;
public:
    RGBEdgeDrawerProcess(bool drawOver);
    virtual bool process(cv::OutputArray out) override;
};







#endif // __RGBEDGEDRAWERPROCESS_H__