#ifndef __IIMAGEPROCESSOR_H__
#define __IIMAGEPROCESSOR_H__

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include "IImageProcess.h"

class IImageProcessor : public IImageProcess
{
public:

    virtual ~IImageProcessor(){}
};

#endif // __IIMAGEPROCESSOR_H__