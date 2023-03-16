#pragma once

#include <ostream>

#include "sample_handler.hpp"

class CsvSampleHandler : public SampleHandler
{
  private:
    std::ostream& _stream;
	
  public:
    CsvSampleHandler(std::ostream &stream);
    
    void HandleSampleData(TofSampler *sampler);
    void Initialize(TofSampler *sampler);
    std::string HandlerName() const;
};
