#pragma once

#include <string>

#include "tof_sampler.hpp"


class SampleHandler
{
  public:
    virtual void HandleSampleData(TofSampler *sampler) = 0;
    virtual void Initialize(TofSampler *sampler) = 0;
    virtual std::string HandlerName() const = 0;
};
