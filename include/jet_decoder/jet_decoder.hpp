#include <opencv2/core/mat.hpp>
#include <optional>
#include <turbojpeg.h>
#include <vector>

namespace jet_decoder
{
class JetDecoder
{
public:
  JetDecoder();

  ~JetDecoder();

  void set_scale(int num, int denom);

  void print_available_scale() const;

  cv::Mat decompress(const std::vector<unsigned char>& jpeg_buf) const;

  cv::Mat decompress_using_cache(const std::vector<unsigned char>& jpeg_buf) const;

private:
  tjscalingfactor scaling_factor_;
  tjhandle tj_instance_ = NULL;
  int num_scaling_factors = 0;
  tjscalingfactor* scaling_factors = NULL;


  struct Cache {
    int dst_width;
    int dst_height;
  };
  mutable std::optional<Cache> cache_{std::nullopt};
};

}  // namespace jet_decoder