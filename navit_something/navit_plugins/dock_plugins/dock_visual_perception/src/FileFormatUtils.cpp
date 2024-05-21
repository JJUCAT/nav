#include "dock_visual_perception/FileFormatUtils.h"
#include <stdio.h>
#include <sstream>
#include <limits>

namespace dock_visual_perception
{
bool FileFormatUtils::decodeXMLMatrix(const TiXmlElement* xml_matrix, int& type,
                                      int& rows, int& cols)
{
  const char* xml_type = xml_matrix->Attribute("type");
  if (strcmp("CV_32F", xml_type) == 0)
    type = CV_32F;
  else if (strcmp("CV_64F", xml_type) == 0)
    type = CV_64F;
  else
    return false;

  if (xml_matrix->QueryIntAttribute("rows", &rows) != TIXML_SUCCESS)
    return false;
  if (xml_matrix->QueryIntAttribute("cols", &cols) != TIXML_SUCCESS)
    return false;

  return true;
}

cv::Mat* FileFormatUtils::allocateXMLMatrix(const TiXmlElement* xml_matrix)
{
  if (!xml_matrix)
    return NULL;

  int type, rows, cols;
  if (!decodeXMLMatrix(xml_matrix, type, rows, cols))
    return NULL;

  return new cv::Mat(rows, cols, type);
}

bool FileFormatUtils::parseXMLMatrix(const TiXmlElement* xml_matrix,
                                     cv::Mat& matrix)
{
  if (!xml_matrix || matrix.empty())
    return false;

  int type, rows, cols;
  if (!decodeXMLMatrix(xml_matrix, type, rows, cols))
    return false;

  if (type != matrix.type())
    return false;
  if (rows != matrix.rows)
    return false;
  if (cols != matrix.cols)
    return false;

  const TiXmlElement* xml_data = xml_matrix->FirstChildElement("data");
  for (int r = 0; r < matrix.rows; ++r)
  {
    for (int c = 0; c < matrix.cols; ++c)
    {
      if (!xml_data)
        return false;
      double value = atof(xml_data->GetText());
      matrix.at<double>(r, c) = value;
      xml_data = (const TiXmlElement*)xml_data->NextSibling("data");
    }
  }

  return true;
}

TiXmlElement* FileFormatUtils::createXMLMatrix(const char* element_name,
                                               const cv::Mat& matrix)
{
  if (matrix.empty())
    return NULL;

  TiXmlElement* xml_matrix = new TiXmlElement(element_name);
  int precision;
  if (matrix.type() == CV_32F)
  {
    xml_matrix->SetAttribute("type", "CV_32F");
    precision = std::numeric_limits<float>::digits10 + 2;
  }
  else if (matrix.type() == CV_64F)
  {
    xml_matrix->SetAttribute("type", "CV_64F");
    precision = std::numeric_limits<double>::digits10 + 2;
  }
  else
  {
    delete xml_matrix;
    return NULL;
  }

  xml_matrix->SetAttribute("rows", matrix.rows);
  xml_matrix->SetAttribute("cols", matrix.cols);

  for (int r = 0; r < matrix.rows; ++r)
  {
    for (int c = 0; c < matrix.cols; ++c)
    {
      TiXmlElement* xml_data = new TiXmlElement("data");
      xml_matrix->LinkEndChild(xml_data);
      std::stringstream ss;
      ss.precision(precision);
      ss << matrix.at<double>(r, c);
      xml_data->LinkEndChild(new TiXmlText(ss.str().c_str()));
    }
  }
  return xml_matrix;
}
}
