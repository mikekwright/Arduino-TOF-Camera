#pragma once

#include <ostream>

#include "sample_handler.hpp"

class OpenCVSampleHandler : public SampleHandler
{
  private:
    int _height;
    int _width;
    uint8_t* _previewPtr;
	
	void getPreview(float *phase_image_ptr, float *amplitude_image_ptr, int maxDistance);
	
  public:
    OpenCVSampleHandler();
    ~OpenCVSampleHandler();
    
    void HandleSampleData(TofSampler *sampler);
    void Initialize(TofSampler *sampler);
    std::string HandlerName() const;
};
