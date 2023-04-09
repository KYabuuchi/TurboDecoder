#include "jet_decoder/jet_decoder.hpp"
#include "jet_decoder/timer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

class Decoder : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  Decoder() : Node("jet_decoder")
  {
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(10).best_effort();
    auto on_compressed_image = std::bind(&Decoder::on_compressed_image, this, _1);
    sub_compressed_image_ = create_subscription<CompressedImage>("/sensing/camera/traffic_light/image_raw/compressed", qos, on_compressed_image);

    decoder_.set_scale(1, 4);
  }

private:
  jet_decoder::JetDecoder decoder_;
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_compressed_image_;

  void on_compressed_image(const CompressedImage& msg)
  {
    jet_decoder::Timer timer;
    cv::Mat image = decoder_.decompress_using_cache(msg.data);
    RCLCPP_INFO_STREAM(get_logger(), "image decompressed: " << timer);

    // cv::imshow("show", image);
    // cv::waitKey(10);
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Decoder>());
  rclcpp::shutdown();
  return 0;
}
