#pragma once
#include <opencv2/imgproc.hpp>

namespace turbo_decoder
{
cv::Mat convert_from_bayer(const cv::Mat src_image, const std::string& format)
{
  cv::Mat dst_image;
  const std::string encoding = format.substr(0, format.find(";"));
  if (encoding == "bayer_rggb8")
    cv::cvtColor(src_image, dst_image, cv::COLOR_BayerBG2BGR);
  else if (encoding == "bayer_bggr8")
    cv::cvtColor(src_image, dst_image, cv::COLOR_BayerRG2BGR);
  else if (encoding == "bayer_grbg8")
    cv::cvtColor(src_image, dst_image, cv::COLOR_BayerGB2BGR);
  else if (encoding == "bayer_gbrg8")
    cv::cvtColor(src_image, dst_image, cv::COLOR_BayerGR2BGR);
  return dst_image;
}

}  // namespace turbo_decoder