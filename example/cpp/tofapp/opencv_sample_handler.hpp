#pragma once

#include <ostream>

#include "sample_handler.hpp"

class OpenCVSampleHandler : public SampleHandler
{
  private:
    int _height;
    int _width;
    int _sampleSize;
    int _sampleHeight;
    int _sampleWidth;
    uint8_t* _previewPtr;
	
  public:
    OpenCVSampleHandler(int sampleSize);
    ~OpenCVSampleHandler();
    void HandleSampleData(TofSampler *sampler);
    std::string HandlerName() const;
};
