#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "opencv_sample_handler.hpp"

#define HEIGHT 180
#define WIDTH 240

using namespace std;


cv::Mat matRotateClockWise180(cv::Mat src);


OpenCVSampleHandler::OpenCVSampleHandler() : _width(-1), _height(-1),
	_previewPtr(nullptr)
{
	cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);
}

OpenCVSampleHandler::~OpenCVSampleHandler()
{
  if (_previewPtr != nullptr) delete [] _previewPtr;
}


void OpenCVSampleHandler::Initialize(TofSampler *sampler)
{
  _width = sampler->Width();
  _height = sampler->Height();
  
  _previewPtr = new uint8_t[_width * _height];
  
  memset(_previewPtr, 0, sizeof(uint8_t) * _width * _height);  
}

string OpenCVSampleHandler::HandlerName() const
{
  return "OpenCVSampleHandler";
}

void OpenCVSampleHandler::HandleSampleData(TofSampler *sampler)
{		
  cout << "Data " << _height << " " << _width << endl;
	
  getPreview(sampler->GetDepthPtr(), sampler->GetAmplitudePtr(), sampler->GetMaxDistance());

  cv::Mat result_frame(_height, _width, CV_8U, _previewPtr);
  cv::Mat depth_frame(_height, _width, CV_32F, sampler->GetDepthPtr());
  cv::Mat amplitude_frame(_height, _width, CV_32F, sampler->GetAmplitudePtr());

  depth_frame = matRotateClockWise180(depth_frame);
  result_frame = matRotateClockWise180(result_frame);
  amplitude_frame = matRotateClockWise180(amplitude_frame);
  
  cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);
  
  amplitude_frame.convertTo(amplitude_frame, CV_8U, 255.0 / 1024, 0);
    
  cv::imshow("amplitude", amplitude_frame);

  cv::imshow("preview", result_frame);
  
  // Waitkey is needed or the image is now shown
  if (cv::waitKey(1) == 27) {
	clog << "Request to stop from opencv image" << endl;
    sampler->Stop();
  }
}


void OpenCVSampleHandler::getPreview(float *phase_image_ptr, float *amplitude_image_ptr, int maxDistance)
{
    auto len = _width * _height;
    for (auto i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(phase_image_ptr + i) / maxDistance)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(_previewPtr + i) = pixel & mask;
    }
}

cv::Mat matRotateClockWise180(cv::Mat src)
{
    if (src.empty())
    {
        std::cerr << "RorateMat src is empty!";
    }

    flip(src, src, 0);
    flip(src, src, 1);
    return src;
}
