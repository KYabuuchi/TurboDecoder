#include "fast_decompress/timer.hpp"
#include "fast_decompress/turbojpeg.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

TurboDecoder decoder;

cv::Mat decompress_image(const sensor_msgs::msg::CompressedImage& compressed_img)
{
  cv::Mat raw_image;

  const std::string& format = compressed_img.format;
  const std::string encoding = format.substr(0, format.find(";"));

  constexpr int DECODE_GRAY = 0;
  constexpr int DECODE_RGB = 1;

  bool encoding_is_bayer = encoding.find("bayer") != std::string::npos;
  if (!encoding_is_bayer) {

    {
      Timer timer;
      cv::Mat mat = decoder.decompress(compressed_img.data);
      std::cout << "jpegjet: " << timer << std::endl;
      return mat;
    }

    {
      Timer timer;
      cv::Mat mat = cv::imdecode(cv::Mat(compressed_img.data), DECODE_RGB);
      cv::resize(mat, mat, cv::Size(), 0.25, 0.25);
      std::cout << "cv::imdecode(): " << timer << std::endl;
      return mat;
    }
  }

  std::cerr << encoding << " is not supported encoding" << std::endl;
  std::cerr << "Please implement additional decoding in " << __FUNCTION__ << std::endl;
  exit(EXIT_FAILURE);
}

namespace fast_decompress
{
class Decoder : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  Decoder() : Node("fast_decompress")
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
    RCLCPP_INFO_STREAM(get_logger(), "msg subscribed");
    cv::Mat image = decompress_image(msg);
    cv::imshow("show", image);
    cv::waitKey(10);
  }
};
}  // namespace fast_decompress


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fast_decompress::Decoder>());
  rclcpp::shutdown();
  return 0;
}
