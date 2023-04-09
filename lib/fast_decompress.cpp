#include "fast_decompress/fast_decompress.hpp"
namespace fast_decompress
{
TurboDecoder::TurboDecoder()
{
  if ((tj_instance_ = tjInitDecompress()) == NULL) {
    std::cerr << tjGetErrorStr2(tj_instance_) << std::endl;
  }

  print_available_scale();

  scaling_factor_.num = 1;
  scaling_factor_.denom = 8;
}

TurboDecoder::~TurboDecoder()
{
  if (tjDestroy(tj_instance_) == -1) {
    std::cerr << tjGetErrorStr2(tj_instance_) << std::endl;
  }
}

void TurboDecoder::print_available_scale() const
{
  int num_scaling_factors = 0;
  tjscalingfactor* scaling_factors = NULL;

  if ((scaling_factors = tjGetScalingFactors(&num_scaling_factors)) == NULL) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }

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

    const int dst_width = TJSCALED(src_width, scaling_factor_);
    const int dst_height = TJSCALED(src_height, scaling_factor_);
    cache_ = Cache{dst_width, dst_height};
  }

  const int dst_width = cache_->dst_width;
  const int dst_height = cache_->dst_height;

  cv::Mat image_buf = cv::Mat(cv::Size(dst_width, dst_height), CV_8UC3);
  const int pixel_format = TJPF_BGR;
  const int flags = TJFLAG_FASTDCT;
  if (tjDecompress2(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), image_buf.data, dst_width, 0, dst_height, pixel_format, flags) < 0) {
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


  cv::Mat image_buf = cv::Mat(cv::Size(dst_width, dst_height), CV_8UC3);
  const int pixel_format = TJPF_BGR;
  const int flags = TJFLAG_FASTDCT;
  if (tjDecompress2(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), image_buf.data, dst_width, 0, dst_height, pixel_format, flags) < 0) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }
  return image_buf;
}


}  // namespace fast_decompress