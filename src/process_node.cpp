#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <mutex>
#include <chrono>

class ContourPoseNode : public rclcpp::Node
{
public:
  ContourPoseNode()
  : Node("contour_pose_node"),
    camera_info_received_(false),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 声明参数
    declare_parameter("physical_width", 0.5);
    declare_parameter("physical_height", 0.5);
    declare_parameter("h_min", 20);
    declare_parameter("h_max", 40);
    declare_parameter("s_min", 100);
    declare_parameter("s_max", 255);
    declare_parameter("v_min", 100);
    declare_parameter("v_max", 255);
    declare_parameter("world_frame", "base_link");

    // 发布器
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);

    RCLCPP_INFO(get_logger(), "ContourPoseNode constructed. Call on_init() after shared_ptr.");
  }

  void on_init()
  {
    // 初始化 image_transport
    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());

    // 订阅
    image_sub_ = it_->subscribe("/robot1/camera/image_raw", 1, &ContourPoseNode::image_callback, this);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/robot1/camera/camera_info", 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info_callback(msg);
      });

    debug_img_pub_ = it_->advertise("/debug/yellow_detection", 1);

    // 启动实时显示定时器（30 FPS）
    display_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      [this]() { display_thread(); }
    );

    // 创建窗口
    cv::namedWindow("Yellow Detection [Realtime]", cv::WINDOW_AUTOSIZE);

    RCLCPP_INFO(get_logger(), "ContourPoseNode fully initialized with real-time display.");
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_info_received_) return;

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        camera_matrix_.at<double>(i, j) = msg->k[i * 3 + j];

    dist_coeffs_ = cv::Mat(1, 5, CV_64F);
    for (size_t i = 0; i < msg->d.size() && i < 5; ++i)
      dist_coeffs_.at<double>(i) = msg->d[i];

    camera_info_received_ = true;
    RCLCPP_INFO(get_logger(), "Camera intrinsics loaded.");
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera info...");
      return;
    }

    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat frame = cv_ptr->image.clone();
      cv::Mat hsv, mask, blurred, binary;

      cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

      int h_min = get_parameter("h_min").as_int();
      int h_max = get_parameter("h_max").as_int();
      int s_min = get_parameter("s_min").as_int();
      int s_max = get_parameter("s_max").as_int();
      int v_min = get_parameter("v_min").as_int();
      int v_max = get_parameter("v_max").as_int();

      cv::inRange(hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);
      cv::GaussianBlur(mask, blurred, cv::Size(5, 5), 0);
      cv::threshold(blurred, binary, 0, 255, cv::THRESH_OTSU);

      auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
      cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
      cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      cv::Mat debug_frame = frame.clone();

      if (!contours.empty()) {
        auto largest = std::max_element(contours.begin(), contours.end(),
          [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });

        if (cv::contourArea(*largest) > 50) {
          cv::RotatedRect rect = cv::minAreaRect(*largest);
          cv::Point2f pts[4]; rect.points(pts);

          float pixel_w = std::max(rect.size.width, rect.size.height);
          double fx = camera_matrix_.at<double>(0, 0);
          double real_w = get_parameter("physical_width").as_double();
          double z = (fx * real_w) / pixel_w;

          double cx = camera_matrix_.at<double>(0, 2);
          double cy = camera_matrix_.at<double>(1, 2);
          double fy = camera_matrix_.at<double>(1, 1);

          double x = (rect.center.x - cx) * z / fx;
          double y = (rect.center.y - cy) * z / fy;

          double angle = rect.angle * CV_PI / 180.0;
          if (rect.size.width < rect.size.height) angle += CV_PI / 2.0;

          // 相机坐标系位姿
          geometry_msgs::msg::PoseStamped cam_pose;
          cam_pose.header = msg->header;
          cam_pose.header.frame_id = "robot1/camera_link";
          cam_pose.pose.position.x = x;
          cam_pose.pose.position.y = y;
          cam_pose.pose.position.z = z;

          double ha = angle / 2.0;
          cam_pose.pose.orientation.w = std::cos(ha);
          cam_pose.pose.orientation.z = std::sin(ha);

          // TF 变换
          try {
            std::string world_frame = get_parameter("world_frame").as_string();
            auto tf = tf_buffer_.lookupTransform(world_frame, "robot1/camera_link", tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped world_pose;
            tf2::doTransform(cam_pose, world_pose, tf);
            pose_pub_->publish(world_pose);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
              "Target: Cam[%.2f,%.2f,%.2f] World[%.2f,%.2f,%.2f]",
              x, y, z,
              world_pose.pose.position.x,
              world_pose.pose.position.y,
              world_pose.pose.position.z);
          } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
          }

          // 绘制调试信息
          for (int i = 0; i < 4; ++i)
            cv::line(debug_frame, pts[i], pts[(i+1)%4], cv::Scalar(0,0,255), 2);
          cv::circle(debug_frame, rect.center, 5, cv::Scalar(255,0,0), -1);
        }
      }

      // 拼接 mask 用于调试
      cv::Mat mask_bgr, combined;
      cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
      cv::hconcat(debug_frame, mask_bgr, combined);

      // 发布调试图像
      if (debug_img_pub_.getNumSubscribers() > 0) {
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = "bgr8";
        out_msg.image = combined;
        debug_img_pub_.publish(out_msg.toImageMsg());
      }

      // 更新实时显示缓冲（线程安全）
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_debug_frame_ = combined.clone();
        new_frame_available_ = true;
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
    }
  }

  // 实时显示线程（由 wall_timer 调用）
  void display_thread()
  {
    cv::Mat frame_to_show;
    bool has_new = false;

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (new_frame_available_) {
        latest_debug_frame_.copyTo(frame_to_show);
        new_frame_available_ = false;
        has_new = true;
      }
    }

    if (has_new) {
      cv::imshow("Yellow Detection [Realtime]", frame_to_show);
      char key = cv::waitKey(1);
      if (key == 'q' || key == 27) {  // q 或 ESC 退出
        RCLCPP_WARN(get_logger(), "Display window closed by user.");
        display_timer_->cancel();
        cv::destroyWindow("Yellow Detection [Realtime]");
      }
    }
  }

  // 成员变量
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher debug_img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  cv::Mat camera_matrix_, dist_coeffs_;
  bool camera_info_received_ = false;

  // 实时显示
  rclcpp::TimerBase::SharedPtr display_timer_;
  cv::Mat latest_debug_frame_;
  std::mutex frame_mutex_;
  bool new_frame_available_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContourPoseNode>();
  node->on_init();
  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}