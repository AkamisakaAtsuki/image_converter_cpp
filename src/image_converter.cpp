#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h> // yaml-cppのインクルード
#include <fstream>
#include <string>

class ImageConverter : public rclcpp::Node
{
public:
  ImageConverter(const std::string &config_file) : Node("image_converter_cpp")
  {
    // YAML設定ファイルを読み込む
    YAML::Node config = YAML::LoadFile(config_file);
    std::string input_topic = config["topics"]["input_topic"].as<std::string>();
    std::string output_topic = config["topics"]["output_topic"].as<std::string>();

    // 設定ファイルからサブスクライブとパブリッシュのトピック名を決定
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic, 10, std::bind(&ImageConverter::image_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // cv_bridgeを使用してROSメッセージからOpenCV画像へ変換
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // カラー画像をモノクロ画像に変換
    cv::Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

    // モノクロ画像をROSメッセージに変換してパブリッシュ
    sensor_msgs::msg::Image::SharedPtr mono_msg = cv_bridge::CvImage(
        msg->header, sensor_msgs::image_encodings::MONO8, gray_image)
        .toImageMsg();
    publisher_->publish(*mono_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  // コマンドライン引数で設定ファイルを受け取る
  if (argc < 2) {
    std::cerr << "Usage: image_converter <config_file>" << std::endl;
    return -1;
  }

  std::string config_file = argv[1];

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageConverter>(config_file);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
