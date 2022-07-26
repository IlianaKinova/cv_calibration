#ifndef __EDGEDETECTPROCESS_H__
#define __EDGEDETECTPROCESS_H__

#include "IMultiResultProcess.h"
#include "opencv2/shape.hpp"

struct EdgeDetectResult
{
    std::vector<std::vector<cv::Point>> Contours;
    std::vector<cv::Vec4i> Hierarchy;
};

class EdgeDetectProcess : public IMultiResultProcess<EdgeDetectResult>
{
private:
    cv::Mat m_canny_output;
    int m_cannyTresh1;
    int m_cannyTresh2;
    cv::RetrievalModes m_mode;
    cv::ContourApproximationModes m_approx;
    volatile bool m_bIsReady = false;
    virtual void run() override;
public:
    // cannyTresh1: Treshold 1 for the canny filter
    // cannyTresh2: Treshold 2 for the canny filter
    // mode: The retrieval mode for the edges (findContours)
    // approx: The approximation algorithm for the edges
    EdgeDetectProcess(int cannyTresh1 = 120, int cannyTresh2 = 120 / 2, cv::RetrievalModes mode = cv::RetrievalModes::RETR_TREE,
        cv::ContourApproximationModes approx = cv::ContourApproximationModes::CHAIN_APPROX_TC89_KCOS) :
        m_cannyTresh1(cannyTresh1), m_cannyTresh2(cannyTresh2), m_mode(mode), m_approx(approx) {}
    virtual bool process(cv::OutputArray out) override;
};



#endif // __EDGEDETECTPROCESS_H__