#pragma once

#include "ArducamTOFCamera.hpp"

class SampleHandler;

class TofSampler
{
  private:
    int _sampleSize;
    int _avgCount;
    int _height;
    int _width;
    int _maxDistance;
    int _sampleAvg;
    int _sampleHeight;
    int _sampleWidth;
    float* _depthRunning;
    float* _amplitudeRunning;
    float* _depthSample;
    float* _amplitudeSample;
    uint8_t* _previewPtr;
    Arducam::ArducamTOFCamera _tof;

  public:
    TofSampler(int sampleSize, int avgCount, int height, int width, int maxDistance);
    ~TofSampler();

    void Start();
    void Capture();
    void RegisterHandler(SampleHandler *handler);

    
  private:
    void ClearSamples();
    void LoadSamples();
    void CalculateSamples();
};
