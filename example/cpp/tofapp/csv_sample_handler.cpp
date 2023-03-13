#include "csv_sample_handler.hpp"

using namespace std;

CsvSampleHandler::CsvSampleHandler(ostream &stream) : _stream(stream)
{}

string CsvSampleHandler::HandlerName() const
{
  return "CsvSampleHandler";
}

void CsvSampleHandler::HandleSampleData(TofSampler *sampler)
{
  _stream << "csv: ";  
  for (int row = 0; row < sampler->Height(); ++row)
  {
    for (int col = 0; col < sampler->Width(); ++col)
    {
	  _stream << sampler->GetDepthValue(row, col) << ",";
    }
  }
  
  _stream << endl;
}
