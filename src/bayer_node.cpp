#include "turbo_decoder/timer.hpp"
#include "turbo_decoder/turbo_decoder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class BayerDecoder : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;

  BayerDecoder() : Node("jet_decoder"),
                   use_imdecode_(declare_parameter<bool>("use_imdecode", false)),
                   use_imshow_(declare_parameter<bool>("use_imshow", false))
  {
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(10).best_effort();
    auto on_compressed_image = std::bind(&BayerDecoder::on_compressed_image, this, _1);
    sub_compressed_image_ = create_subscription<CompressedImage>("src_image", qos, on_compressed_image);

    {
      const int num = declare_parameter<int>("scale_num");
      const int denom = declare_parameter<int>("scale_denom");
      decoder_.set_scale(num, denom);
      scale_ratio_ = static_cast<double>(num) / static_cast<double>(denom);
    }

    decoder_.set_gray();
  }

private:
  const bool use_imdecode_;
  const bool use_imshow_;
  double scale_ratio_;
  turbo_decoder::TurboDecoder decoder_;
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_compressed_image_;

  void on_compressed_image(const CompressedImage& msg)
  {
    jet_decoder::Timer timer;
    cv::Mat image;

    if (use_imdecode_) {
      const std::string& format = msg.format;
      const std::string encoding = format.substr(0, format.find(";"));
      image = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_GRAYSCALE);
      if (encoding == "bayer_rggb8")
        cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
      else if (encoding == "bayer_bggr8")
        cv::cvtColor(image, image, cv::COLOR_BayerRG2BGR);
      else if (encoding == "bayer_grbg8")
        cv::cvtColor(image, image, cv::COLOR_BayerGB2BGR);
      else if (encoding == "bayer_gbrg8")
        cv::cvtColor(image, image, cv::COLOR_BayerGR2BGR);
      cv::resize(image, image, cv::Size(), scale_ratio_, scale_ratio_);
    } else {
      image = decoder_.decompress_using_cache(msg.data);
      cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
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
  rclcpp::spin(std::make_shared<BayerDecoder>());
  rclcpp::shutdown();
  return 0;
}