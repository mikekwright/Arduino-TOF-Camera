#pragma once

#include "tof_sampler.hpp"


class SampleHandler
{
  public:
    virtual void HandleSampleData(TofSampler *sampler) = 0;
};
