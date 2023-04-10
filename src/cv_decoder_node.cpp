#include "turbo_decoder/timer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Decoder : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  Decoder() : Node("cv_decoder")
  {
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(10).best_effort();
    auto on_compressed_image = std::bind(&Decoder::on_compressed_image, this, _1);
    sub_compressed_image_ = create_subscription<CompressedImage>("/sensing/camera/traffic_light/image_raw/compressed", qos, on_compressed_image);
  }

private:
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_compressed_image_;

  void on_compressed_image(const CompressedImage& msg)
  {
    jet_decoder::Timer timer;
    cv::Mat image = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_COLOR);
    cv::resize(image, image, cv::Size(), 1.0 / 4, 1.0 / 4);
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
