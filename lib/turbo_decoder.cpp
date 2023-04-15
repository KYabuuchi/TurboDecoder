#include "turbo_decoder/turbo_decoder.hpp"
#include "turbo_decoder/timer.hpp"
#include <iostream>

namespace turbo_decoder
{
TurboDecoder::TurboDecoder()
{
  if ((tj_instance_ = tjInitDecompress()) == NULL) {
    std::cerr << tjGetErrorStr2(tj_instance_) << std::endl;
  }

  if ((tj_instance_ = tjInitTransform()) == NULL) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }

  scaling_factor_.num = 1;
  scaling_factor_.denom = 1;
}

TurboDecoder::~TurboDecoder()
{
  if (tjDestroy(tj_instance_) == -1) {
    std::cerr << tjGetErrorStr2(tj_instance_) << std::endl;
  }
}

void TurboDecoder::set_scale(int num, int denom)
{
  scaling_factor_.num = num;
  scaling_factor_.denom = denom;

  // Get avalibale scaling factors
  tjscalingfactor* scaling_factors = NULL;
  int num_scaling_factors;
  if ((scaling_factors = tjGetScalingFactors(&num_scaling_factors)) == NULL) {
    throw std::runtime_error(tjGetErrorStr2(tj_instance_));
  }

  // Try to match query wtih avalibale scaling factors
  for (int i = 0; i < num_scaling_factors; i++) {
    if ((scaling_factors[i].num == num) && (scaling_factors[i].denom == denom)) {
      return;  // success
    }
  }

  // Print available scaling factors
  {
    for (int i = 0; i < num_scaling_factors; i++) {
      std::cout << scaling_factors[i].num << "/" << scaling_factors[i].denom << ", ";
    }
    std::cout << std::endl;
  };

  throw std::runtime_error("invalid scale");
}

void TurboDecoder::set_crop_range(int x, int y, int w, int h)
{
  if ((x % 16 != 0) || (y % 16 != 0) || (w % 16 != 0) || (h % 16 != 0)) {
    std::cerr << "crop range must be consisted by multiple of 16" << std::endl;
    std::cerr << "e.g. x=16, y=32, w=640, h=480" << std::endl;
    throw std::runtime_error("invalid crop range");
  }

  tjtransform xform{};
  xform.r.x = x;
  xform.r.y = y;
  xform.r.w = w;
  xform.r.h = h;
  xform.options |= TJXOPT_TRIM;
  xform.options |= TJXOPT_CROP;
  xform_ = xform;
}

cv::Mat TurboDecoder::decompress(const std::vector<unsigned char>& jpeg_buf)
{
  const int flags = TJFLAG_FASTDCT;  // NOTE:

  int src_width, src_height;
  int src_sub_sample, src_color_space;

  unsigned char* dst_buf = NULL;
  unsigned long dst_size = 0;
  if (xform_) {
    if (tjTransform(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), 1, &dst_buf, &dst_size, &(*xform_), flags) < 0) {
      tjFree(dst_buf);
      throw std::runtime_error(tjGetErrorStr2(tj_instance_));
    }

    if (tjDecompressHeader3(tj_instance_, dst_buf, dst_size, &src_width, &src_height, &src_sub_sample, &src_color_space) < 0) {
      throw std::runtime_error(tjGetErrorStr2(tj_instance_));
    }

  } else {
    if (tjDecompressHeader3(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), &src_width, &src_height, &src_sub_sample, &src_color_space) < 0) {
      throw std::runtime_error(tjGetErrorStr2(tj_instance_));
    }
  }

  const int dst_width = TJSCALED(src_width, scaling_factor_);
  const int dst_height = TJSCALED(src_height, scaling_factor_);

  cv::Mat image_buf = create_image_buffer(dst_width, dst_height);

  if (xform_) {
    if (tjDecompress2(tj_instance_, dst_buf, dst_size, image_buf.data, dst_width, 0, dst_height, pixel_format_, flags) < 0) {
      throw std::runtime_error(tjGetErrorStr2(tj_instance_));
    }
  } else {
    if (tjDecompress2(tj_instance_, jpeg_buf.data(), jpeg_buf.size(), image_buf.data, dst_width, 0, dst_height, pixel_format_, flags) < 0) {
      throw std::runtime_error(tjGetErrorStr2(tj_instance_));
    }
  }


  return image_buf;
}

}  // namespace turbo_decoder