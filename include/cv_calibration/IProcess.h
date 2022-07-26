#ifndef __IPROCESS_H__
#define __IPROCESS_H__

#include <thread>

class IProcess
{
private:
protected:
    static void threadRun(IProcess* self)
    {
        while (self->m_isRunning)
        {
            self->run();
        }
    }
    std::thread* m_process;
    bool m_isRunning = false;

    virtual void run() {}

public:
    IProcess() = default;
    IProcess(const IProcess&) = default;
    IProcess(IProcess&&) = default;

    virtual bool start()
    {
        if (!m_isRunning)
        {
            m_isRunning = true;
            m_process = new std::thread(threadRun, this);
        }
    }
    virtual bool stop()
    {
        m_isRunning = false;
        m_process->join();
        if (m_process) delete m_process;
    }
    
    virtual ~IProcess()
    {
        if (m_process) delete m_process;
    }

    friend class ProcessingChain;
};

#endif // __IPROCESS_H__