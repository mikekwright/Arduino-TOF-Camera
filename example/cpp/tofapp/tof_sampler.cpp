#include <iostream>
#include <cstring>


#include "ArducamTOFCamera.hpp"
#include "tof_sampler.hpp"
#include "sample_handler.hpp"


using namespace Arducam;
using namespace std;

#define HEIGHT 180
#define WIDTH 240

// This can be either 2 or 4, 2 is 2 meters and 4 is 4 meters (range)
//    NOTE: For some reason when run with 2 meters get some odd results
#define MAX_DISTANCE 4


TofSampler::TofSampler(int pixelBinningSize, int numFramesToIntegrate) :
        _pixelBinningSize(pixelBinningSize), 
        _numberOfFramesToIntegrate(numFramesToIntegrate), 
        _maxDistance(MAX_DISTANCE),
        _running(false)
{
  _sampleAvg = _pixelBinningSize * _pixelBinningSize;
  _sampleHeight = HEIGHT / _pixelBinningSize;
  _sampleWidth = WIDTH / _pixelBinningSize;

  _depthRunning = new float[HEIGHT * WIDTH];
  _amplitudeRunning = new float[HEIGHT * WIDTH];

  _depthSample = new float[_sampleHeight * _sampleWidth];
  _amplitudeSample = new float[_sampleHeight * _sampleWidth];  
  
  clog << "Initialized Sampler: " << endl;
  clog << "\tPixel Bin Size: " << _pixelBinningSize << endl;
  clog << "\tFrames to Integrate: " << _numberOfFramesToIntegrate << endl;
  clog << "\tFinal Height: " << _sampleHeight << endl;
  clog << "\tFinal Width: " << _sampleWidth << endl;
  clog << "\tMaxDistance: " << _maxDistance << endl;
}


TofSampler::~TofSampler()
{  
  delete [] _depthRunning;
  delete [] _amplitudeRunning;
  delete [] _depthSample;
  delete [] _amplitudeSample;
}

void TofSampler::RegisterHandler(SampleHandler* handler)
{
  clog << "Registering handler: " << handler->HandlerName() << endl;
  handler->Initialize(this);
  _handlers.push_back(handler);
}


void TofSampler::ClearSamples()
{
  clog << "Clearing Samples" << endl;
  memset(_depthRunning, 0.0f, sizeof(float) * HEIGHT * WIDTH);
  memset(_amplitudeRunning, 0.0f, sizeof(float) * HEIGHT * WIDTH);
}


void TofSampler::LoadSamples()
{
  clog << "Loading Samples" << endl;
  
  int count = 0;
  ArducamFrameBuffer *frame;
  
  float* depthPtr;
  float* amplitudePtr;

  for (int i = 0; i < (HEIGHT * WIDTH); ++i) {
    _depthRunning[i] = 0.0f;
    _amplitudeRunning[i] = 0.0f;
  }

  do
  {
    frame = _tof.requestFrame(200);
    if (frame != nullptr)
    {
      depthPtr = (float *)frame->getData(FrameType::DEPTH_FRAME);
      amplitudePtr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);
      // Is this memory being freed?

      // Add to common pool
      for (int i = 0; i < (HEIGHT * WIDTH); ++i) {
        _depthRunning[i] += depthPtr[i];
        _amplitudeRunning[i] += amplitudePtr[i];
      }
    }
    _tof.releaseFrame(frame);
  } while (++count < this->_numberOfFramesToIntegrate);
}


void TofSampler::CalculateSamples()
{
  clog << "Calculating Samples" << endl;
  
  for (int row = 0; row < _sampleHeight; row++) {
    for (int col = 0; col < _sampleWidth; col++) {
      // This is where we do our small rectangle averaging
      int largeRow = row * _pixelBinningSize;
      int largeCol = col * _pixelBinningSize;

      float depthValue = 0.0f;
      float ampValue = 0.0f;
      for (int lr = largeRow; lr < (largeRow + _pixelBinningSize); lr++) {
        for (int lc = largeCol; lc < (largeCol + _pixelBinningSize); lc++) {
          depthValue += _depthRunning[(lr * WIDTH) + lc];
          ampValue += _amplitudeRunning[(lr * WIDTH) + lc];
        }
      }

      _depthSample[(row * _sampleWidth) + col] = depthValue / _sampleAvg;
      _amplitudeSample[(row * _sampleWidth) + col] = ampValue / _sampleAvg;
    }
  }
}

void TofSampler::SupplySamplesToHandlers()
{
  clog << "Applying sample handlers" << endl;
  
  for (auto iter = _handlers.begin(); iter != _handlers.end(); ++iter)
  {
    clog << "Applying handler: " << (*iter)->HandlerName() << endl;
    (*iter)->HandleSampleData(this);
  }
}


int TofSampler::Height() const { return _sampleHeight; }
int TofSampler::Width() const { return _sampleWidth; }
int TofSampler::SampleSize() const { return _pixelBinningSize; }
int TofSampler::GetMaxDistance() const { return _maxDistance; }
float* TofSampler::GetDepthPtr() { return _depthSample; }
float* TofSampler::GetAmplitudePtr() { return _amplitudeSample; }

float TofSampler::GetDepthValue(int row, int col) const
{
  return _depthSample[(row * _sampleHeight) + col];
}

float TofSampler::GetAmplitudeValue(int row, int col) const
{
  return _amplitudeSample[(row * _sampleHeight) + col];
}



void TofSampler::Capture(int count)
{
  clog << "Starting capture" << endl;
  _running = true;
  
  int runningCount = 0;
  
  do
  {
    ClearSamples();
    LoadSamples();  
    CalculateSamples();
    SupplySamplesToHandlers();
    
    if (count != 0 && ++runningCount == count) {
      _running = false;
    }      
  } while (_running);
}

bool TofSampler::Stop()
{
  if (_running == false) {
    return true;
  }
  
  _running = false;
  
  if (_tof.stop()) {
    return false;
  }
  
  return true;
}

void TofSampler::Start()
{
  if (_running)
  {
    clog << "Cannot start sampler, already running" << endl;
    return;
  }
  
  
  clog << "Starting Sampler" << endl;
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
  //_tof.setControl(ControlID::RANGE, 2);

  memset(_depthRunning, 0.0f, sizeof(float) * HEIGHT * WIDTH);
  memset(_amplitudeRunning, 0.0f, sizeof(float) * HEIGHT * WIDTH);
  memset(_depthSample, 0.0f, sizeof(float) * _sampleHeight * _sampleWidth);
  memset(_amplitudeSample, 0.0f, sizeof(float) * _sampleHeight * _sampleWidth);
}
