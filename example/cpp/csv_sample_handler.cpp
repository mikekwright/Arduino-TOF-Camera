#include "csv_sample_handler.hpp"

using namespace std;

CsvSampleHandler::CsvSampleHandler(ostream &stream) : _stream(stream)
{}

void CsvSampleHandler::HandleSampleData(TofSampler *sampler)
{
	_stream << "Hi" << endl;
}
