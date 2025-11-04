#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <thread>
#include <mutex>

// This was made entirely by chatgpt, quality is unknown
class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
      : Node("camera_node"), frame_counter_(0), running_(true)
  {
    // --- Parameters ---
    this->declare_parameter<bool>("simulation_mode", true);
    this->declare_parameter<double>("fps", 60.0);

    sim_mode_ = this->get_parameter("simulation_mode").as_bool();
    fps_ = this->get_parameter("fps").as_double();
    if (fps_ <= 0.0)
      fps_ = 60.0;

    // --- QoS Profile ---
    rclcpp::QoS qos(1);
    qos.best_effort();
    qos.durability_volatile();
    qos.lifespan(rclcpp::Duration(1, static_cast<int>(1e9 / fps_)));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);

    // --- Initialize OpenCV ---
    if (sim_mode_)
    {
      std::string video_path = "/ws/data/EVC_test_footage/video.mp4";
      cap_.open(video_path);
      if (!cap_.isOpened())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s", video_path.c_str());
      }
      video_fps_ = cap_.get(cv::CAP_PROP_FPS);
      reader_thread_ = std::thread(&CameraNode::read_frames, this);
    }
    else
    {
      cap_.open(0);
    }

    RCLCPP_INFO(this->get_logger(), "Video FPS: %.2f, Given FPS: %.2f", video_fps_, fps_);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / fps_),
        std::bind(&CameraNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "CPP Camera Node started - Mode: %s",
                sim_mode_ ? "SIM" : "REAL");
  }

  ~CameraNode() override
  {
    running_ = false;
    if (reader_thread_.joinable())
      reader_thread_.join();
    cap_.release();
  }

private:
  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_)
    {
      image_pub_->publish(*latest_frame_);
      frame_counter_++;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No frame yet, skipping");
    }
  }

  void read_frames()
  {
    cv_bridge::CvImage bridge;
    const double frame_time = 1.0 / video_fps_;

    while (rclcpp::ok() && running_)
    {
      auto start = std::chrono::steady_clock::now();
      cv::Mat frame;
      if (!cap_.read(frame))
      {
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        continue;
      }

      // Resize
      int width = 360;
      int height = static_cast<int>(frame.rows * (360.0 / frame.cols));
      cv::resize(frame, frame, cv::Size(width, height));

      // Convert to ROS image
      bridge.encoding = "bgr8";
      bridge.image = frame;
      bridge.header.stamp = this->now();
      bridge.header.frame_id = "camera";

      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = bridge.toImageMsg();
      }

      auto elapsed = std::chrono::steady_clock::now() - start;
      auto sleep_time = std::chrono::duration<double>(frame_time) - elapsed;
      if (sleep_time.count() > 0)
        std::this_thread::sleep_for(sleep_time);
    }
  }

  // --- Members ---
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  double fps_ = 60.0;
  double video_fps_ = 60.0;
  bool sim_mode_ = true;
  bool running_;
  size_t frame_counter_;
  std::shared_ptr<sensor_msgs::msg::Image> latest_frame_;
  std::mutex frame_mutex_;
  std::thread reader_thread_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
