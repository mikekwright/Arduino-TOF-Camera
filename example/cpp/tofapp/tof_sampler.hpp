#pragma once

#include <vector>
#include "ArducamTOFCamera.hpp"

class SampleHandler;


class TofSampler
{
  private:
    bool _running;
    int _pixelBinningSize;
    int _numberOfFramesToIntegrate;
    int _maxDistance;
    int _sampleAvg;
    int _sampleHeight;
    int _sampleWidth;
    float* _depthRunning;
    float* _amplitudeRunning;
    float* _depthSample;
    float* _amplitudeSample;
    Arducam::ArducamTOFCamera _tof;
    std::vector<SampleHandler*> _handlers;

  public:
    TofSampler(int pixelBinningSize, int numFramesToIntegrate);
    ~TofSampler();

    void Start();
    bool Stop();
    void Capture(int count = 0);
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
