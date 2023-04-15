#include "turbo_decoder/timer.hpp"
#include "turbo_decoder/turbo_decoder.hpp"

#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class DemoNode : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  DemoNode() : Node("turbo_decoder")
  {
    using std::placeholders::_1;
    auto qos = rclcpp::QoS(10).best_effort();
    auto on_compressed_image = std::bind(&DemoNode::on_compressed_image, this, _1);
    sub_compressed_image_ = create_subscription<CompressedImage>("src_image", qos, on_compressed_image);

    {
      const int num = declare_parameter<int>("scale_num");
      const int denom = declare_parameter<int>("scale_denom");
      decoder_.set_scale(num, denom);
      scale_ratio_ = static_cast<double>(num) / static_cast<double>(denom);
    }
  }

private:
  double scale_ratio_;
  turbo_decoder::TurboDecoder decoder_;
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_compressed_image_;

  void on_compressed_image(const CompressedImage& msg)
  {
    // NOTE: Place data on cache
    // (The first process is slower because there is no data on the cache.)
    cv::imdecode(cv::Mat(msg.data), cv::IMREAD_COLOR);

    // (1) turbo_decoder
    turbo_decoder::Timer timer;
    cv::Mat image1 = decoder_.decompress(msg.data);
    const long time1 = timer.micro_seconds();

    // (2) cv::imdecode
    timer.reset();
    cv::Mat image2 = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_COLOR);
    cv::resize(image2, image2, cv::Size(), scale_ratio_, scale_ratio_);
    const long time2 = timer.micro_seconds();


    // Visualize
    auto put_text = [](cv::Mat image, long time, const std::string& prefix) -> void {
      std::stringstream ss;
      ss << prefix << " " << std::fixed << std::setprecision(3) << (time / 1000.0) << " [ms]";
      cv::putText(image, ss.str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255), 2);
    };

    put_text(image1, time1, "turbo_decode");
    put_text(image2, time2, "cv::imdecode");
    RCLCPP_INFO_STREAM(get_logger(), "turbo: " << time1 / 1000. << " imdecode:" << time2 / 1000.);

    if (image1.size() != image2.size()) {
      using namespace std::literals::chrono_literals;
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "scale does not match");
      cv::resize(image1, image1, image2.size());
    }

    cv::Mat concat_image;
    cv::hconcat(image1, image2, concat_image);
    cv::imshow("show", concat_image);
    cv::waitKey(10);
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoNode>());
  rclcpp::shutdown();
  return 0;
}
