#include "turbo_decoder/cvt_color.hpp"
#include "turbo_decoder/timer.hpp"
#include "turbo_decoder/turbo_decoder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Decoder : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;

  Decoder() : Node("turbo_decoder"),
              use_imdecode_(declare_parameter<bool>("use_imdecode", false)),
              use_imshow_(declare_parameter<bool>("use_imshow", false)),
              is_bayer_(declare_parameter<bool>("is_bayer", false))
  {
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(10).best_effort();
    auto on_compressed_image = std::bind(&Decoder::on_compressed_image, this, _1);
    sub_compressed_image_ = create_subscription<CompressedImage>("src_image", qos, on_compressed_image);

    {
      const int num = declare_parameter<int>("scale_num");
      const int denom = declare_parameter<int>("scale_denom");
      decoder_.set_scale(num, denom);
      scale_ratio_ = static_cast<double>(num) / static_cast<double>(denom);
    }

    {
      declare_parameter<std::vector<int>>("crop_range");
      std::vector<int64_t> range = get_parameter("crop_range").as_integer_array();
      if (range.size() == 4) {
        decoder_.set_crop_range(range[0], range[1], range[2], range[3]);
        crop_range_ = cv::Rect2i(range[0], range[1], range[2], range[3]);
      }
    }

    if (is_bayer_) {
      decoder_.set_gray();
    }
  }

private:
  const bool use_imdecode_;
  const bool use_imshow_;
  const bool is_bayer_;
  double scale_ratio_;
  std::optional<cv::Rect2i> crop_range_{std::nullopt};
  turbo_decoder::TurboDecoder decoder_;
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_compressed_image_;

  void on_compressed_image(const CompressedImage& msg)
  {
    turbo_decoder::Timer timer;
    cv::Mat image;

    if (use_imdecode_) {
      if (is_bayer_) {
        image = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_GRAYSCALE);
        image = turbo_decoder::convert_from_bayer(image, msg.format);
      } else {
        image = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_COLOR);
      }

      if (crop_range_) {
        image = image(crop_range_.value());
      }
      cv::resize(image, image, cv::Size(), scale_ratio_, scale_ratio_);
    } else {
      image = decoder_.decompress(msg.data);
      if (is_bayer_) {
        image = turbo_decoder::convert_from_bayer(image, msg.format);
      }
    }

    RCLCPP_INFO_STREAM(get_logger(), "image decompressed: " << timer);

    if (use_imshow_) {
      cv::imshow("decompressed", image);
      cv::waitKey(10);
    }
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Decoder>());
  rclcpp::shutdown();
  return 0;
}
