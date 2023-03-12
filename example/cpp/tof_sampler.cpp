#include <iostream>
#include <cstring>

#include "ArducamTOFCamera.hpp"
#include "tof_sampler.hpp"


using namespace Arducam;
using namespace std;


TofSampler::TofSampler(int sampleSize, int avgCount, int height, int width, int maxDistance) :
        _sampleSize(sampleSize), _avgCount(avgCount), _height(height), _width(width), _maxDistance(maxDistance)
{
  _sampleAvg = _sampleSize ^ 2;
  _sampleHeight = _height / _sampleSize;
  _sampleWidth = _width / _sampleSize;

  _depthRunning = new float(height * width);
  _amplitudeRunning = new float(height * width);

  _depthSample = new float(_sampleHeight * _sampleWidth);
  _amplitudeSample = new float(_sampleHeight * _sampleWidth);
  
  _previewPtr = new uint8_t(_sampleHeight * _sampleWidth);
}


TofSampler::~TofSampler()
{
  delete [] _depthRunning;
  delete [] _amplitudeRunning;
  delete [] _depthSample;
  delete [] _amplitudeSample;
  delete [] _previewPtr;
}


void TofSampler::ClearSamples()
{
  memset(_depthRunning, 0.0f, sizeof(float) * _height * _width);
  memset(_amplitudeRunning, 0.0f, sizeof(float) * _height * _width);
}


void TofSampler::LoadSamples()
{
  int count = 0;
  ArducamFrameBuffer *frame;
  
  float* depthPtr;
  float* amplitudePtr;

  do
  {
    frame = _tof.requestFrame(200);
    if (frame != nullptr)
    {
      depthPtr = (float *)frame->getData(FrameType::DEPTH_FRAME);
      amplitudePtr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);
      // Is this memory being freed?

      // Add to common pool
      for (int i = 0; i < (_height * _width); ++i) {
        _depthRunning[i] += depthPtr[i];
        _amplitudeRunning[i] += amplitudePtr[i];
      }
    }
    _tof.releaseFrame(frame);
  } while (count++ < this->_avgCount);  
}


void TofSampler::CalculateSamples()
{
  for (int row = 0; row < _sampleHeight; row++) {
    for (int col = 0; col < _sampleWidth; col++) {
      // This is where we do our small rectangle averaging
      int largeRow = row * _sampleSize;
      int largeCol = col * _sampleSize;

      float depthValue = 0.0f;
      float ampValue = 0.0f;
      for (int lr = largeRow; lr < largeRow + _sampleSize; lr++) {
        for (int lc = largeCol; lc < largeCol + _sampleSize; lc++) {
          depthValue += _depthRunning[(lr * _width) + lc];
          ampValue += _amplitudeRunning[(lr * _width) + lc];
        }
      }

      _depthSample[(row * _sampleWidth) + col] = depthValue / _sampleAvg;
      _amplitudeSample[(row * _sampleWidth) + col] = ampValue / _sampleAvg;
    }
  }
}


void TofSampler::Capture()
{
  ClearSamples();
  LoadSamples();  
  CalculateSamples();
  
  //RenderSample();
  
  
  // for (int row = 0; row < SAMPLE_HEIGHT; row++) {
  //   for (int col = 0; col < SAMPLE_WIDTH; col++) {
  //     // This is where we do our small rectangle averaging
  //     int large_row = row * SAMPLE_SIZE;
  //     int large_col = col * SAMPLE_SIZE;

  //     float depth_value = 0.0f;
  //     float amp_value = 0.0f;
  //     for (int lr = large_row; lr < large_row + SAMPLE_SIZE; lr++) {
  //       for (int lc = large_col; lc < large_col + SAMPLE_SIZE; lc++) {
  //         depth_value += depth_running[(lr * WIDTH) + lc];
  //         amp_value += amplitude_running[(lr * WIDTH) + lc];
  //       }
  //     }

  //     depth_sample[(row * SAMPLE_WIDTH) + col] = depth_value / SAMPLE_AVG;
  //     amplitude_sample[(row * SAMPLE_WIDTH) + col] = amp_value / SAMPLE_AVG;
  //   }
  // }

  // //depth_ptr = depth_running;
  // depth_ptr = depth_sample;
  // //amplitude_ptr = amplitude_running;
  // amplitude_ptr = amplitude_sample;
}


void TofSampler::Start()
{
  if (_tof.init(Connection::CSI))
  {
      std::cerr << "initialization failed" << std::endl;
      exit(-1);
  }

  if (_tof.start(FrameType::DEPTH_FRAME))
  {
      std::cerr << "Failed to start camera" << std::endl;
      exit(-1);
  }

  _tof.setControl(ControlID::RANGE, this->_maxDistance);

  memset(_depthRunning, 0.0f, sizeof(float) * _height * _width);
  memset(_amplitudeRunning, 0.0f, sizeof(float) * _height * _width);
  memset(_depthSample, 0.0f, sizeof(float) * _sampleHeight * _sampleWidth);
  memset(_amplitudeSample, 0.0f, sizeof(float) * _sampleHeight * _sampleWidth);
  memset(_previewPtr, 0, sizeof(uint8_t) * _sampleHeight * _sampleWidth);
}
