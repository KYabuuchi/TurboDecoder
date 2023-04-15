#include <opencv2/core/mat.hpp>
#include <optional>
#include <turbojpeg.h>
#include <vector>

namespace turbo_decoder
{
class TurboDecoder
{
public:
  TurboDecoder();

  ~TurboDecoder();

  void set_scale(int num, int denom);

  void set_crop_range(int x, int y, int w, int h);

  void set_gray()
  {
    pixel_format_ = TJPF_GRAY;
  }

  cv::Mat decompress(const std::vector<unsigned char>& jpeg_buf);

private:
  tjscalingfactor scaling_factor_;
  tjhandle tj_instance_ = NULL;

  int pixel_format_ = TJPF_BGR;

  std::optional<tjtransform> xform_{std::nullopt};

  cv::Mat create_image_buffer(int width, int height) const
  {
    cv::Mat image_buf;
    if (pixel_format_ == TJPF_GRAY) {
      image_buf = cv::Mat(cv::Size(width, height), CV_8UC1);
    } else {
      image_buf = cv::Mat(cv::Size(width, height), CV_8UC3);
    }
    return image_buf;
  }
};

}  // namespace turbo_decoder