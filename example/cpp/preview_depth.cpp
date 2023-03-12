#include "ArducamTOFCamera.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iostream>
#include <fstream>

#include "tof_sampler.hpp"

// MAX_DISTANCE value modifiable  is 2 or 4
#define MAX_DISTANCE 4
using namespace Arducam;
using namespace std;

#define HEIGHT 180
#define WIDTH 240

#define SAMPLE_SIZE 6
#define AVG_COUNT 10

#define SAMPLE_AVG (SAMPLE_SIZE * SAMPLE_SIZE)
#define SAMPLE_HEIGHT HEIGHT / SAMPLE_SIZE
#define SAMPLE_WIDTH WIDTH / SAMPLE_SIZE

void display_fps(void)
{
    static int count = 0;
    ++count;
    static std::chrono::high_resolution_clock::time_point time_beg = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::ratio<1, 1>> duration_s = time_end - time_beg;
    if (duration_s.count() >= 1)
    {
        std::cout << "fps:" << count << std::endl;
        count = 0;
        time_beg = time_end;
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

void getPreview(uint8_t *preview_ptr, float *phase_image_ptr, float *amplitude_image_ptr)
{
    auto len = SAMPLE_WIDTH * SAMPLE_HEIGHT;
    for (auto i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(phase_image_ptr + i) / MAX_DISTANCE)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
}

cv::Rect seletRect(0, 0, 0, 0);
cv::Rect followRect(0, 0, 0, 0);

void onMouse(int event, int x, int y, int flags, void *param)
{
    if (x < 4 || x > 251 || y < 4 || y > 251)
        return;
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:

        break;

    case cv::EVENT_LBUTTONUP:
        seletRect.x = x - 4 ? x - 4 : 0;
        seletRect.y = y - 4 ? y - 4 : 0;
        seletRect.width = 8;
        seletRect.height = 8;
        break;
    default:
        followRect.x = x - 4 ? x - 4 : 0;
        followRect.y = y - 4 ? y - 4 : 0;
        followRect.width = 8;
        followRect.height = 8;
        break;
    }
}

int main(int argc, char* argv[])
{
    bool use_csv = false;
    if (argc == 2) {
      if (argv[1][0] == 'c') {
        cout << "Request to use CSV" << endl;
        use_csv = true;
      }
    }
    
    TofSampler x(1, 2, 3, 4, 5);



    ArducamTOFCamera tof;
    ArducamFrameBuffer *frame;
    if (tof.init(Connection::CSI))
    {
        std::cerr << "initialization failed" << std::endl;
        exit(-1);
    }

    if (tof.start(FrameType::DEPTH_FRAME))
    {
        std::cerr << "Failed to start camera" << std::endl;
        exit(-1);
    }
    //  Modify the range also to modify the MAX_DISTANCE
    tof.setControl(ControlID::RANGE, MAX_DISTANCE);
    //CameraInfo tofFormat = tof.getCameraInfo();

    float *depth_ptr;
    float *amplitude_ptr;
    uint8_t *preview_ptr = new uint8_t[SAMPLE_HEIGHT * SAMPLE_WIDTH];
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("preview", onMouse);

    float depth_running[HEIGHT * WIDTH];
    float amplitude_running[HEIGHT * WIDTH];

    float depth_sample[SAMPLE_HEIGHT * SAMPLE_WIDTH];
    float amplitude_sample[SAMPLE_HEIGHT * SAMPLE_WIDTH];

    memset(depth_running, 0.0f, sizeof(depth_running));
    memset(amplitude_running, 0.0f, sizeof(amplitude_running));
    memset(depth_sample, 0.0f, sizeof(depth_sample));
    memset(amplitude_sample, 0.0f, sizeof(amplitude_sample));

    ofstream csv("output.csv");

    int count = 0;
    int framecount = 0;
    for (;;)
    {
        count += 1;	
        frame = tof.requestFrame(200);
        if (frame != nullptr)
        {
            depth_ptr = (float *)frame->getData(FrameType::DEPTH_FRAME);
            amplitude_ptr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);
            // Is this memory being freed?

            // Add to common pool
            for (int i = 0; i < (HEIGHT * WIDTH); ++i) {
              depth_running[i] += depth_ptr[i];
              amplitude_running[i] += amplitude_ptr[i];
            }
        }
        tof.releaseFrame(frame);

        if (count > AVG_COUNT) {
            for (int row = 0; row < SAMPLE_HEIGHT; row++) {
              for (int col = 0; col < SAMPLE_WIDTH; col++) {
                // This is where we do our small rectangle averaging
                int large_row = row * SAMPLE_SIZE;
                int large_col = col * SAMPLE_SIZE;

                float depth_value = 0.0f;
                float amp_value = 0.0f;
                for (int lr = large_row; lr < large_row + SAMPLE_SIZE; lr++) {
                  for (int lc = large_col; lc < large_col + SAMPLE_SIZE; lc++) {
                    depth_value += depth_running[(lr * WIDTH) + lc];
                    amp_value += amplitude_running[(lr * WIDTH) + lc];
                  } 
                }

                depth_sample[(row * SAMPLE_WIDTH) + col] = depth_value / SAMPLE_AVG;
                amplitude_sample[(row * SAMPLE_WIDTH) + col] = amp_value / SAMPLE_AVG;
              }
            }

            //depth_ptr = depth_running;
            depth_ptr = depth_sample;
            //amplitude_ptr = amplitude_running;
            amplitude_ptr = amplitude_sample;

            if (use_csv) {
              cout << "Outputting CSV frame: " << framecount << endl;
              for (int row = 0; row < SAMPLE_HEIGHT; ++row) {
                csv << framecount << "," << row;
                for (int col = 0; col < SAMPLE_WIDTH; ++col) {
                  csv << "," << depth_ptr[row * SAMPLE_HEIGHT + col];
                }
                csv << endl;
              }
              framecount += 1;
            }
            else {
              cout << "Sharing OpenCV image" << endl;
                    getPreview(preview_ptr, depth_ptr, amplitude_ptr);

                    cv::Mat result_frame(SAMPLE_HEIGHT, SAMPLE_WIDTH, CV_8U, preview_ptr);
                    cv::Mat depth_frame(SAMPLE_HEIGHT, SAMPLE_WIDTH, CV_32F, depth_ptr);
                    cv::Mat amplitude_frame(SAMPLE_HEIGHT, SAMPLE_WIDTH, CV_32F, amplitude_ptr);

                    depth_frame = matRotateClockWise180(depth_frame);
                    result_frame = matRotateClockWise180(result_frame);
                    amplitude_frame = matRotateClockWise180(amplitude_frame);
                    
                    cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);
                    amplitude_frame.convertTo(amplitude_frame, CV_8U, 255.0 / 1024, 0);
                    cv::imshow("amplitude", amplitude_frame);
                    cv::rectangle(result_frame, seletRect, cv::Scalar(0, 0, 0), 2);
                    cv::rectangle(result_frame, followRect, cv::Scalar(255, 255, 255), 1);

                    std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << std::endl;

                    cv::imshow("preview", result_frame);
            }

            if (cv::waitKey(1) == 27) {
              if (tof.stop()) {
                exit(-1);
              }

              exit(0);
            }
            display_fps();

            memset(depth_running, 0.0f, sizeof(depth_running));
            memset(amplitude_running, 0.0f, sizeof(amplitude_running));
            count = 0;
        }
    }

    if (tof.stop())
        exit(-1);
    return 0;
}
