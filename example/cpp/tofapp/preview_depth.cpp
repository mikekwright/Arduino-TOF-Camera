#include "ArducamTOFCamera.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iostream>
#include <fstream>

#include "tof_sampler.hpp"
#include "csv_sample_handler.hpp"
#include "opencv_sample_handler.hpp"

// MAX_DISTANCE value modifiable  is 2 or 4

using namespace Arducam;
using namespace std;



#define SAMPLE_SIZE 6
#define AVG_COUNT 10

#define SAMPLE_AVG (SAMPLE_SIZE * SAMPLE_SIZE)
#define SAMPLE_HEIGHT HEIGHT / SAMPLE_SIZE
#define SAMPLE_WIDTH WIDTH / SAMPLE_SIZE

int main(int argc, char* argv[])
{
    bool use_csv = false;
    if (argc == 2) {
      if (argv[1][0] == 'c') {
        cout << "Request to use CSV" << endl;
        use_csv = true;
      }
    }
    
    TofSampler x(1, 1);
    
    CsvSampleHandler csvHandler(cout);
    OpenCVSampleHandler opencvHandler(1);

    x.RegisterHandler(&csvHandler);
    x.RegisterHandler(&opencvHandler);
    x.Start();
    x.Capture();

    if (!x.Stop()) {
        return -1;
    }

    return 0;
}
