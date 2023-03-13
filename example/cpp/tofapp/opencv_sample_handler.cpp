#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "opencv_sample_handler.hpp"


using namespace std;

void getPreview(uint8_t *preview_ptr, float *phase_image_ptr, float *amplitude_image_ptr,
    int sampleWidth, int sampleHeight, int maxDistance);
cv::Mat matRotateClockWise180(cv::Mat src);


OpenCVSampleHandler::OpenCVSampleHandler(int width, int height, int sampleSize) :
	_width(width), _height(height), _sampleSize(sampleSize)
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
  //cv::rectangle(result_frame, seletRect, cv::Scalar(0, 0, 0), 2);
  //cv::rectangle(result_frame, followRect, cv::Scalar(255, 255, 255), 1);

  //clog << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << std::endl;
  clog << "Rendering preview" << endl;
  cv::imshow("preview", result_frame);
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
