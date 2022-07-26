#ifndef __IMULTIRESULTPROCESS_H__
#define __IMULTIRESULTPROCESS_H__

#include "IImageProcess.h"
#include <vector>

class IMultiResultProcessConverter : public IImageProcess
{
};

// Base class for processes with multiple results
template <typename T>
class IMultiResultProcess : public IBaseImageProcess
{
public:
    class ToImageProcessConverter : public IMultiResultProcessConverter
    {
    private:
        std::vector<IMultiResultProcess<T>*> m_chain;
        volatile bool m_ready = false;
    protected:
        virtual void run() override {} // Since the processing is done by other threads, no need for a 
    public:
        virtual ToImageProcessConverter& operator>(IMultiResultProcess<T>& other) 
        {
            m_chain.push_back(&other);
            return *this;
        }
        virtual bool setInput(cv::InputArray in) override
        {
            if (m_chain.size() != 0) // If there are items in the chain
            {
                m_chain.at(0)->setInput(in);
            }
        }
        virtual bool process(cv::OutputArray out) override
        {
            for (size_t i = 0, ii = 1; ii < m_chain.size(); i++, ii++)
            {
                *(m_chain.at(i))>*(m_chain.at(ii));
            }
            m_ready = true;
            if (m_chain.size() != 0) // If there are items in the chain
            {
                m_chain.at(m_chain.size() - 1)->process(out);
            }
        }
        virtual bool start() override
        {
            if (!m_isRunning)
            {
                m_isRunning = true;
                for (auto p : m_chain)
                {
                    p->start();
                }
            }
            return true;
        }

        // Stop processes
        virtual bool stop() override
        {
            // Stopping in reverse order
            m_isRunning = false;
            for (auto p = m_chain.rbegin(); p < m_chain.rend(); p++)
            {
                (*p)->stop();
            }
            return true;
        }
    };
protected:
    T m_payload;
public:
    IMultiResultProcess() {}
    // Get the payload data
    virtual T& getPayload() {return m_payload;}
    // Set the incoming payload data
    virtual void setPayload(const T& in) {m_payload = in;}

    virtual IMultiResultProcess<T>& operator>(IMultiResultProcess<T>& other)
    {
        cv::Mat m;
        if (process(m))
        {
            other.setPayload(m_payload);
            other.setInput(m);
        }
    }
};


#endif // __IMULTIRESULTPROCESS_H__