#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "opencv_sample_handler.hpp"

#define HEIGHT 180
#define WIDTH 240

using namespace std;


void getPreview(uint8_t *preview_ptr, float *phase_image_ptr, float *amplitude_image_ptr,
    int sampleWidth, int sampleHeight, int maxDistance);
cv::Mat matRotateClockWise180(cv::Mat src);


OpenCVSampleHandler::OpenCVSampleHandler(int sampleSize) :
	_width(WIDTH), _height(HEIGHT), _sampleSize(sampleSize)
{
  cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);
  
  _sampleHeight = _height / _sampleSize;
  _sampleWidth = _width / _sampleSize;
  _previewPtr = new uint8_t[_sampleHeight * _sampleWidth];
  memset(_previewPtr, 0, sizeof(uint8_t) * _sampleHeight * _sampleWidth);
}

OpenCVSampleHandler::~OpenCVSampleHandler()
{
  delete [] _previewPtr;
}

string OpenCVSampleHandler::HandlerName() const
{
  return "OpenCVSampleHandler";
}

void OpenCVSampleHandler::HandleSampleData(TofSampler *sampler)
{	
  getPreview(_previewPtr, sampler->GetDepthPtr(), sampler->GetAmplitudePtr(),
	_sampleWidth, _sampleHeight, sampler->GetMaxDistance());

  cv::Mat result_frame(_sampleHeight, _sampleWidth, CV_8U, _previewPtr);
  cv::Mat depth_frame(_sampleHeight, _sampleWidth, CV_32F, sampler->GetDepthPtr());
  cv::Mat amplitude_frame(_sampleHeight, _sampleWidth, CV_32F, sampler->GetAmplitudePtr());

  depth_frame = matRotateClockWise180(depth_frame);
  result_frame = matRotateClockWise180(result_frame);
  amplitude_frame = matRotateClockWise180(amplitude_frame);
                    
  cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);
  amplitude_frame.convertTo(amplitude_frame, CV_8U, 255.0 / 1024, 0);
  cv::imshow("amplitude", amplitude_frame);

  clog << "Rendering preview" << endl;
  cv::imshow("preview", result_frame);
  
  if (cv::waitKey(1) == 27) {
	//if (tof.stop()) {
    //  exit(-1);
    //}

    sampler->Stop();          
  }
}


void getPreview(uint8_t *preview_ptr, float *phase_image_ptr, float *amplitude_image_ptr,
    int sampleWidth, int sampleHeight, int maxDistance)
{
    auto len = sampleWidth * sampleHeight;
    for (auto i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(phase_image_ptr + i) / maxDistance)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
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
