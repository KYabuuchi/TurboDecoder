#include "turbo_decoder/from_bayer.hpp"
#include "turbo_decoder/timer.hpp"
#include "turbo_decoder/turbo_decoder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/float32.hpp>

class Decoder : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using Float32 = std_msgs::msg::Float32;

  Decoder() : Node("turbo_decoder"),
              use_imdecode_(declare_parameter<bool>("use_imdecode", false)),
              use_imshow_(declare_parameter<bool>("use_imshow", false)),
              is_bayer_(declare_parameter<bool>("is_bayer", false))
  {
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(10).best_effort();
    auto on_compressed_image = std::bind(&Decoder::on_compressed_image, this, _1);
    sub_compressed_image_ = create_subscription<CompressedImage>("src_image", qos, on_compressed_image);
    pub_proc_time_ = create_publisher<Float32>("proc_time", 10);

    if (is_bayer_) {
      decoder_.set_gray();
    }

    // Parameters for scaling
    {
      const int num = declare_parameter<int>("scale_num");
      const int denom = declare_parameter<int>("scale_denom");
      decoder_.set_scale(num, denom);
      scale_ratio_ = static_cast<double>(num) / static_cast<double>(denom);
    }

    // Parameters for cropping
    {
      declare_parameter<std::vector<int>>("crop_range");
      std::vector<int64_t> range = get_parameter("crop_range").as_integer_array();
      if (range.size() == 4) {
        decoder_.set_crop_range(range[0], range[1], range[2], range[3]);
        crop_range_ = cv::Rect2i(range[0], range[1], range[2], range[3]);
      }
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
  rclcpp::Publisher<Float32>::SharedPtr pub_proc_time_;
  int sum_count_ = 0;
  int sum_time_us_ = 0;

  void on_compressed_image(const CompressedImage& msg)
  {
    turbo_decoder::Timer timer;
    cv::Mat image;
    if (use_imdecode_)
      image = decompress_by_imdecode(msg);
    else
      image = decompress_by_turbo(msg);

    sum_time_us_ += timer.micro_seconds();
    sum_count_++;
    RCLCPP_INFO_STREAM(get_logger(), "average of " << sum_count_ << " times: " << (sum_time_us_ * 1e-3f / sum_count_) << " ms");

    {
      Float32 time_msg;
      time_msg.data = timer.micro_seconds() / 1000.0;
      pub_proc_time_->publish(time_msg);
    }

    if (use_imshow_) {
      cv::imshow("decompressed", image);
      cv::waitKey(10);
    }
  }

  cv::Mat decompress_by_turbo(const CompressedImage& msg)
  {
    cv::Mat image = decoder_.decompress(msg.data);
    if (is_bayer_) {
      image = turbo_decoder::convert_from_bayer(image, msg.format);
    }
    return image;
  }

  cv::Mat decompress_by_imdecode(const CompressedImage& msg)
  {
    cv::Mat image;
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
    return image;
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Decoder>());
  rclcpp::shutdown();
  return 0;
}
