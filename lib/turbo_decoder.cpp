#include "turbo_decoder/turbo_decoder.hpp"
#include <iostream>

namespace turbo_decoder
{
TurboDecoder::TurboDecoder()
{
  if ((tj_instance_ = tjInitDecompress()) == NULL) {
    std::cerr << tjGetErrorStr2(tj_instance_) << std::endl;
  }
  scaling_factor_.num = 1;
  scaling_factor_.denom = 1;

  if ((scaling_factors = tjGetScalingFactors(&num_scaling_factors)) == NULL) {
    std::cerr << tjGetErrorStr2(tj_instance_) << std::endl;
  }

  if ((tj_instance_ = tjInitTransform()) == NULL) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }
}

void TurboDecoder::set_scale(int num, int denom)
{
  scaling_factor_.num = num;
  scaling_factor_.denom = denom;

  for (int i = 0; i < num_scaling_factors; i++) {
    const int ref_num = scaling_factors[i].num;
    const int ref_denom = scaling_factors[i].denom;

    if ((ref_num == num) && (ref_denom == denom)) {
      return;
    }
  }

  print_available_scale();
  throw std::runtime_error("invalid scale");
}

TurboDecoder::~TurboDecoder()
{
  if (tjDestroy(tj_instance_) == -1) {
    std::cerr << tjGetErrorStr2(tj_instance_) << std::endl;
  }
}

void TurboDecoder::print_available_scale() const
{
  for (int i = 0; i < num_scaling_factors; i++) {
    std::cout << scaling_factors[i].num << "/" << scaling_factors[i].denom << ", ";
  }
  std::cout << std::endl;
}

cv::Mat TurboDecoder::decompress_using_cache(const std::vector<unsigned char>& jpeg_buf) const
{
  if (!cache_) {
    int src_width, src_height;
    int src_sub_sample, src_color_space;
    if (tjDecompressHeader3(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), &src_width, &src_height, &src_sub_sample, &src_color_space) < 0) {
      throw std::runtime_error(tjGetErrorStr2(tj_instance_));
    }

    cache_ = Cache{};
    cache_->dst_width = TJSCALED(src_width, scaling_factor_);
    cache_->dst_height = TJSCALED(src_height, scaling_factor_);
  }

  const int dst_width = cache_->dst_width;
  const int dst_height = cache_->dst_height;

  cv::Mat image_buf;
  if (pixel_format_ == TJPF_GRAY) {
    image_buf = cv::Mat(cv::Size(dst_width, dst_height), CV_8UC1);
  } else {
    image_buf = cv::Mat(cv::Size(dst_width, dst_height), CV_8UC3);
  }

  const int flags = TJFLAG_FASTDCT;  // NOTE:
  if (tjDecompress2(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), image_buf.data, dst_width, 0, dst_height, pixel_format_, flags) < 0) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }

  return image_buf;
}

cv::Mat TurboDecoder::decompress_crop(const std::vector<unsigned char>& jpeg_buf)
{
  unsigned char* dst_buf = NULL;
  unsigned long dst_size = 0;

  const int flags = TJFLAG_FASTDCT;  // NOTE:

  int src_width, src_height;
  int src_sub_sample, src_color_space;

  if (xform_) {
    if (tjTransform(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), 1, &dst_buf, &dst_size, &(*xform_), flags) < 0) {
      tjFree(dst_buf);
      throw std::runtime_error(tjGetErrorStr2(tj_instance_));
    }

    if (tjDecompressHeader3(tj_instance_, dst_buf, dst_size, &src_width, &src_height, &src_sub_sample, &src_color_space) < 0) {
      throw std::runtime_error(tjGetErrorStr2(tj_instance_));
    }
  }
  // TODO:
  //  else {
  //   if (tjDecompressHeader3(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), &src_width, &src_height, &src_sub_sample, &src_color_space) < 0) {
  //     throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  //   }
  //   dst_buf = jpeg_buf.data();
  //   dst_size = jpeg_buf.size();
  // }

  const int dst_width = TJSCALED(src_width, scaling_factor_);
  const int dst_height = TJSCALED(src_height, scaling_factor_);

  cv::Mat image_buf;
  if (pixel_format_ == TJPF_GRAY) {
    image_buf = cv::Mat(cv::Size(dst_width, dst_height), CV_8UC1);
  } else {
    image_buf = cv::Mat(cv::Size(dst_width, dst_height), CV_8UC3);
  }

  if (tjDecompress2(tj_instance_, dst_buf, dst_size, image_buf.data, dst_width, 0, dst_height, pixel_format_, flags) < 0) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }
  return image_buf;
}

cv::Mat TurboDecoder::decompress(const std::vector<unsigned char>& jpeg_buf) const
{
  int src_width, src_height;
  int src_sub_sample, src_color_space;
  if (tjDecompressHeader3(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), &src_width, &src_height, &src_sub_sample, &src_color_space) < 0) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }

  const int dst_width = TJSCALED(src_width, scaling_factor_);
  const int dst_height = TJSCALED(src_height, scaling_factor_);

  cv::Mat image_buf;
  if (pixel_format_ == TJPF_GRAY) {
    image_buf = cv::Mat(cv::Size(dst_width, dst_height), CV_8UC1);
  } else {
    image_buf = cv::Mat(cv::Size(dst_width, dst_height), CV_8UC3);
  }

  const int flags = TJFLAG_FASTDCT;  // NOTE:
  if (tjDecompress2(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), image_buf.data, dst_width, 0, dst_height, pixel_format_, flags) < 0) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }
  return image_buf;
}


}  // namespace turbo_decoder