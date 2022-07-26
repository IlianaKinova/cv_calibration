#ifndef __COLORDEPTHSINGLEFRAMECAPURE_H__
#define __COLORDEPTHSINGLEFRAMECAPURE_H__

#include "IImageProcess.h"
#include "IAcquisition.h"
#include "opencv2/highgui.hpp"


class ColorDepthSingleFrameCapture : public IProcess
{
public:
    class SingleFrameCapture : public IAcquisition
    {
    private:
        volatile bool m_first = true;
        void setImage(cv::InputArray in) 
        { 
            lock(m_lock)
            {
                in.copyTo(m_output);
                m_first = false;
                m_result.Done = false;
            } 
        }
    protected:
        virtual void run() override {} // No need to run a thread here
    public:
        virtual bool process(cv::OutputArray out) 
        {
            if (m_first) return false; // First image isn't available yet
            lock(m_lock)
            {
                m_output.copyTo(out);
            }
            return true;
        }
        virtual IImageProcess& operator>>(IImageProcess& other)
        {
            if (m_result.Done) resetTimeout();
            m_result.Success = m_result.Success && process(m_output) && other.setInput(m_output);
            passResultTo(other);
            return other;
        }
        
        virtual bool start() override { return true; } // We do not need to start a thread
        virtual bool stop() override { return true; } // We do not need to stop a thread

        friend class ColorDepthSingleFrameCapture;
    };
private:
    std::string m_colorAddress;
    std::string m_depthAddress;
    SingleFrameCapture m_colorCapture;
    SingleFrameCapture m_depthCapture;
    std::mutex& m_colorLock = m_colorCapture.m_lock;
    std::mutex& m_depthLock = m_depthCapture.m_lock;
    cv::VideoCapture m_colorStream;
    cv::VideoCapture m_depthStream;
    volatile bool m_capture = false;
protected:
    virtual void run() override;

public:
    ColorDepthSingleFrameCapture(std::string colorAddress, std::string depthAddress);

    SingleFrameCapture& getColorCapture();
    SingleFrameCapture& getDepthCapture();
    void capture() { m_capture = true; }

};



#endif // __COLORDEPTHSINGLEFRAMECAPURE_H__