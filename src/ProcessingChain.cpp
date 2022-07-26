#include "ProcessingChain.h"


void ProcessingChain::run()
{
    for (size_t i = 0, ii = 1; ii < m_chain.size(); i++, ii++)
    {
        *(m_chain.at(i))>>*(m_chain.at(ii));
    }
}

ProcessingChain& ProcessingChain::operator>>(IImageProcess& other)
{
    m_chain.push_back(&other);
    return *this;
}

bool ProcessingChain::start()
{
    if (!m_isRunning)
    {
        m_isRunning = true;
        m_process = new std::thread(threadRun, this);
        for (auto p : m_chain)
        {
            p->start();
        }
    }
}

bool ProcessingChain::stop()
{
    m_isRunning = false;
    m_process->join();
    if (m_process) delete m_process;
    for (auto p = m_chain.rbegin(); p < m_chain.rend(); p++)
    {
        (*p)->stop();
    }
    
}