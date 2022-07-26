#ifndef __DISPLAYPROCESS_H__
#define __DISPLAYPROCESS_H__

#include "IImageProcessor.h"

class DisplayProcess : public IImageProcessor
{
private:
    std::string m_name;
    volatile bool m_ready = false;
protected:
    virtual void run() override;
public:
    DisplayProcess(std::string name);
    DisplayProcess(const DisplayProcess&) = default;
    DisplayProcess(DisplayProcess&&) = default;
    virtual bool process(cv::OutputArray out) override;
    virtual bool setInput(cv::InputArray in) override;
};





#endif // __DISPLAYPROCESS_H__