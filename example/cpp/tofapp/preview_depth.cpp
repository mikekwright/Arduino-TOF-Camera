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

using namespace Arducam;
using namespace std;

//
// Pulled from Stackoverflow (Thanks @iain)
//   https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
class InputParser{
    public:
        InputParser (int &argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
        }
        /// @author iain
        const std::string& getCmdOption(const std::string &option) const{
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()){
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }
        /// @author iain
        bool cmdOptionExists(const std::string &option) const{
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }
    private:
        std::vector <std::string> tokens;
};


void displayUsage(string &programName, bool error = false)
{
    cout << endl;
    cout << "Usage: " << programName << " [FLAGS | OPTIONS]" << endl;
    cout << "    " << "FLAGS" << endl;
    cout << "\t" << "-h:\t Display this help message" << endl;
    cout << "\t" << "-c:\t Output CSV data to stdout (must have csv or opencv)" << endl;
    cout << "\t" << "-p:\t Open an OpenCV image preview (must have csv or opencv)" << endl;
    
    cout << "    " << "OPTIONS" << endl;        
    cout << "\t" << "-b <n>:\t Specify the number of pixels to bin together (1 to 30)" << endl;
    cout << "\t" << "-s <n>:\t Specify the number of frames to integrate over" << endl;
    cout << "\t" << "-m <n>:\t Specify the number of iterations to run (default infinite)" << endl;
    cout << endl;
    
    exit(!error);
}


int main(int argc, char** argv)
{
    InputParser inputs(argc, argv);
    
    string programName = argv[0];
    bool use_csv = false;
    bool use_opencv = false;
    int pixel_bin_count = 1;
    int frames_to_integrate = 1;
    int run_count = 0;

    if (inputs.cmdOptionExists("-h"))
    {
        displayUsage(programName);
    }
    
    if (inputs.cmdOptionExists("-c"))
    {
        use_csv = true;
    }
    
    if (inputs.cmdOptionExists("-p"))
    {
        use_opencv = true;
    }
    
    if (inputs.cmdOptionExists("-b"))
    {
        const string &pixelBinStr = inputs.getCmdOption("-b");
        pixel_bin_count = stoi(pixelBinStr);
        cout << pixel_bin_count << endl;
        if (pixel_bin_count <= 0 || pixel_bin_count > 30) {
            cerr << "Please specify a valid bin count between 1 and 30" << endl;
            displayUsage(programName, true);
        }    
    }
    
    if (inputs.cmdOptionExists("-s"))
    {
        const string &frameCount = inputs.getCmdOption("-s");
        frames_to_integrate = stoi(frameCount);
        if (frames_to_integrate <= 0) {
            cerr << "Please specify a valid frames to integrate (1+)" << endl;
            displayUsage(programName, true);
        }
    }
    
    if (inputs.cmdOptionExists("-m"))
    {
        const string &maxCount = inputs.getCmdOption("-m");
        run_count = stoi(maxCount);
        if (run_count <= 0) {
            cerr << "Please specify a valid run count (0+)" << endl;
            displayUsage(programName, true);
        }
    }
    
    if (!use_csv && !use_opencv)
    {
        cerr << "Please specify either cvs or opencv" << endl;
        displayUsage(programName, true);
    }
    
    TofSampler x(pixel_bin_count, frames_to_integrate);
    
    CsvSampleHandler csvHandler(cout);
    OpenCVSampleHandler opencvHandler;

    if (use_csv) x.RegisterHandler(&csvHandler);
    if (use_opencv) x.RegisterHandler(&opencvHandler);
    
    x.Start();
    x.Capture(run_count);

    if (!x.Stop()) {
        return -1;
    }

    return 0;
}

