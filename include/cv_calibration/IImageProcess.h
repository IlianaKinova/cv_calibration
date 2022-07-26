#ifndef __IIMAGEPROCESS_H__
#define __IIMAGEPROCESS_H__

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <ros/ros.h>
#include "lock.h"
#include <thread>
#include "IProcess.h"

class IBaseImageProcess;

struct ImageProcessResult
{
    bool Success = true;
    bool Done = true;
    IBaseImageProcess* EntryPoint = nullptr;
};

class IBaseImageProcess : public IProcess
{
protected:
    std::mutex m_lock;
    cv::Mat m_input;
    cv::Mat m_output;
    ImageProcessResult m_result;

    // Passes the curent result to the specified process
    void passResultTo(IBaseImageProcess& other) { other.m_result = m_result; }
public:
    virtual bool setInput(cv::InputArray in) 
    {
        lock(m_lock)
        {
            in.copyTo(m_input);
        }
        return true;
    }
    virtual bool currentImage(cv::OutputArray out)
    {
        lock(m_lock)
        {
            m_output.copyTo(out);
        }
        return true;
    }
    virtual bool process(cv::OutputArray out) = 0;
};

class IImageProcess : public IBaseImageProcess
{
public:
    IImageProcess() = default;
    IImageProcess(const IImageProcess&) = default;
    IImageProcess(IImageProcess&&) = default;

    IImageProcess& operator>>(IImageProcess& other)
    {
        m_result.Success = m_result.Success && process(m_output) && other.setInput(m_output);
        passResultTo(other);
        return other;
    }

    virtual ~IImageProcess(){}
};

#endif // __IIMAGEPROCESS_H__