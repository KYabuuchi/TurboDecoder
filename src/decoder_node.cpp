#include "jet_decoder/jet_decoder.hpp"
#include "jet_decoder/timer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Decoder : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  Decoder() : Node("jet_decoder"), use_jet_(declare_parameter<bool>("use_jet", true))
  {
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(10).best_effort();
    auto on_compressed_image = std::bind(&Decoder::on_compressed_image, this, _1);
    sub_compressed_image_ = create_subscription<CompressedImage>("/sensing/camera/traffic_light/image_raw/compressed", qos, on_compressed_image);

    decoder_.set_scale(1, 8);
  }

private:
  jet_decoder::JetDecoder decoder_;
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_compressed_image_;
  const bool use_jet_;

  void on_compressed_image(const CompressedImage& msg)
  {
    Timer timer;
    cv::Mat image = decompress_image(msg);
    RCLCPP_INFO_STREAM(get_logger(), "image decompressed: " << timer);

    // cv::imshow("show", image);
    // cv::waitKey(10);
  }

  cv::Mat decompress_image(const sensor_msgs::msg::CompressedImage& compressed_img)
  {
    if (use_jet_) {
      cv::Mat mat = decoder_.decompress_using_cache(compressed_img.data);
      return mat;
    } else {
      constexpr int DECODE_RGB = 1;
      cv::Mat mat = cv::imdecode(cv::Mat(compressed_img.data), DECODE_RGB);
      cv::resize(mat, mat, cv::Size(), 0.25, 0.25);
      return mat;
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
