#ifndef __PROCESSINGCHAIN_H__
#define __PROCESSINGCHAIN_H__


#include <vector>
#include "IImageProcess.h"


class ProcessingChain : public IProcess
{
private:
    std::vector<IImageProcess*> m_chain;
protected:
    virtual void run() override;
public:
    ProcessingChain() = default;
    ProcessingChain(const ProcessingChain&) = default;
    ProcessingChain(ProcessingChain&&) = default;
    ProcessingChain& operator>>(IImageProcess& other);
    virtual bool start() override;
    virtual bool stop() override;
};


#endif // __PROCESSINGCHAIN_H__