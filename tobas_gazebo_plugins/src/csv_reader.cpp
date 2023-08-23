#pragma once

#include <fstream>
#include <sstream>
#include <iostream>

#include "../include/tobas_gazebo_plugins/csv_reader.hpp"

using namespace std;

namespace gazebo
{
bool CsvReader::readCsvFile(string file_name, vector<vector<double>>& datas)
{
  fstream file_stream;
  file_stream.open(file_name, ios::in);
  if (file_stream.is_open())
  {
    string header;
    getline(file_stream, header, '\n');
    while (!file_stream.eof())
    {
      string line_str;
      getline(file_stream, line_str, '\n');
      stringstream line_stream;
      line_stream << line_str;
      vector<double> data;
      try
      {
        while (!line_stream.eof())
        {
          string value;
          getline(line_stream, value, ',');
          data.push_back(stod(value));
        }
      }
      catch (...)
      {
        cerr << "cannot convert str:" << line_str << endl;
        continue;
      }
      datas.push_back(data);
    }
    cerr << "data size:" << datas.size() << endl;
    return true;
  }
  else
  {
    cerr << "cannot read csv file!" << file_name << endl;
  }
  return false;
}
}  // namespace gazebo
