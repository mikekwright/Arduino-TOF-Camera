#pragma once

#include <vector>
#include "ArducamTOFCamera.hpp"

class SampleHandler;


class TofSampler
{
  private:
    bool _running;
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
    std::vector<SampleHandler*> _handlers;

  public:
    TofSampler(int sampleSize, int avgCount, int height, int width, int maxDistance);
    ~TofSampler();

    void Start();
    void Stop();
    void Capture();
    void RegisterHandler(SampleHandler* handler);
    
    int Height() const;
    int Width() const;
    int SampleSize() const;
    int GetMaxDistance() const;
    float GetDepthValue(int row, int col) const;
    float GetAmplitudeValue(int row, int col) const;
    float* GetDepthPtr();
    float* GetAmplitudePtr();
    
    
  private:
    void ClearSamples();
    void LoadSamples();
    void CalculateSamples();
    void SupplySamplesToHandlers();
};
