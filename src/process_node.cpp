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

class ContourPoseNode : public rclcpp::Node
{
public:
  ContourPoseNode() : Node("contour_pose_node"), camera_info_received_(false), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    this->declare_parameter("physical_width", 0.5);  // 物体的物理宽度（米）
    this->declare_parameter("physical_height", 0.5); // 物体的物理高度（米）

    this->declare_parameter("h_min", 20);   
    this->declare_parameter("h_max", 40);   
    this->declare_parameter("s_min", 100); 
    this->declare_parameter("s_max", 255);  
    this->declare_parameter("v_min", 100);  
    this->declare_parameter("v_max", 255);  
    this->declare_parameter("world_frame", "odom"); // 世界坐标系名称

    image_transport::ImageTransport it(shared_from_this());
    image_sub_ = it.subscribe("/mogi_bot/camera/image_raw", 1, &ContourPoseNode::image_callback, this);
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/mogi_bot/camera/camera_info", 10,
      std::bind(&ContourPoseNode::camera_info_callback, this, std::placeholders::_1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
    debug_img_pub_ = it.advertise("/debug/yellow_detection", 1);

    RCLCPP_INFO(this->get_logger(), "Contour Pose Node started. Waiting for camera info and images...");
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_info_received_) {
      return;
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        camera_matrix_.at<double>(i, j) = msg->k[i * 3 + j];
      }
    }

    dist_coeffs_ = cv::Mat(1, 5, CV_64F);
    for (size_t i = 0; i < msg->d.size() && i < 5; ++i) {
      dist_coeffs_.at<double>(i) = msg->d[i];
    }

    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera parameters received and initialized.");
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                          "Waiting for camera info...");
      return;
    }

    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat frame = cv_ptr->image;
      cv::Mat hsv, yellow_mask, blurred, binary;

      // 1. 转换为HSV颜色空间
      cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

      // 2. 获取黄色HSV阈值参数
      int h_min = this->get_parameter("h_min").as_int();
      int h_max = this->get_parameter("h_max").as_int();
      int s_min = this->get_parameter("s_min").as_int();
      int s_max = this->get_parameter("s_max").as_int();
      int v_min = this->get_parameter("v_min").as_int();
      int v_max = this->get_parameter("v_max").as_int();

      // 3. 提取黄色区域
      cv::Scalar lower_yellow(h_min, s_min, v_min);
      cv::Scalar upper_yellow(h_max, s_max, v_max);
      cv::inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

      // 4. 图像预处理
      cv::GaussianBlur(yellow_mask, blurred, cv::Size(5, 5), 0); 
      cv::threshold(blurred, binary, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); 

      // 5. 形态学操作
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
      cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
      cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

      // 6. 查找轮廓
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // 7. 筛选最大轮廓
      if (!contours.empty()) {
        double max_area = 0;
        std::vector<cv::Point> largest_contour;

        for (const auto& contour : contours) {
          double area = cv::contourArea(contour);
          if (area > max_area && area > 50) {
            max_area = area;
            largest_contour = contour;
          }
        }

        if (!largest_contour.empty()) {
          // 拟合最小面积矩形
          cv::RotatedRect min_rect = cv::minAreaRect(largest_contour);
          
          // 获取矩形的四个角点
          cv::Point2f rect_points[4];
          min_rect.points(rect_points);

          // 计算矩形的宽度和高度（像素）
          float pixel_width = std::max(min_rect.size.width, min_rect.size.height);

          // 使用相机内参计算距离
          double fx = camera_matrix_.at<double>(0, 0);
          double physical_width = this->get_parameter("physical_width").as_double();
          double distance_z = (fx * physical_width) / pixel_width;

          // 计算中心点在图像坐标系中的位置
          cv::Point2f center = min_rect.center;
          double center_x = center.x;
          double center_y = center.y;

          // 计算在相机坐标系中的3D位置
          double principal_x = camera_matrix_.at<double>(0, 2);
          double principal_y = camera_matrix_.at<double>(1, 2);
          double fy = camera_matrix_.at<double>(1, 1);

          double world_x = (center_x - principal_x) * distance_z / fx;
          double world_y = (center_y - principal_y) * distance_z / fy;

          // 计算旋转角度
          double angle = min_rect.angle * CV_PI / 180.0;
          if (min_rect.size.width < min_rect.size.height) {
            angle += CV_PI / 2.0;
          }

          // 创建相机坐标系下的位姿
          auto camera_pose = geometry_msgs::msg::PoseStamped();
          camera_pose.header = msg->header;
          camera_pose.header.frame_id = "camera_frame";

          camera_pose.pose.position.x = world_x;
          camera_pose.pose.position.y = world_y;
          camera_pose.pose.position.z = distance_z;

          // 四元数表示绕Z轴的旋转
          double half_angle = angle / 2.0;
          camera_pose.pose.orientation.w = cos(half_angle);
          camera_pose.pose.orientation.x = 0.0;
          camera_pose.pose.orientation.y = 0.0;
          camera_pose.pose.orientation.z = sin(half_angle);

          // 转换到世界坐标系
          try {
            std::string world_frame = this->get_parameter("world_frame").as_string();
            
            // 获取从相机坐标系到世界坐标系的变换
            auto transform = tf_buffer_.lookupTransform(
              world_frame, 
              camera_pose.header.frame_id,
              tf2::TimePointZero);
            
            // 应用变换
            geometry_msgs::msg::PoseStamped world_pose;
            tf2::doTransform(camera_pose, world_pose, transform);
            
            // 发布世界坐标系下的位姿
            pose_pub_->publish(world_pose);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Detected yellow object - Camera frame: [x:%.3f y:%.3f z:%.3f] World frame: [x:%.3f y:%.3f z:%.3f]",
                                world_x, world_y, distance_z,
                                world_pose.pose.position.x, world_pose.pose.position.y, world_pose.pose.position.z);

          } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF transformation failed: %s", ex.what());
            return;
          }

          // 调试绘图
          if (debug_drawing_) {
            // 绘制最小面积矩形
            for (int j = 0; j < 4; j++) {
              cv::line(frame, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0, 0, 255), 2);
            }
            // 绘制中心点
            cv::circle(frame, center, 5, cv::Scalar(255, 0, 0), -1);
            // 显示黄色掩码
            cv::Mat mask_vis;
            cv::cvtColor(yellow_mask, mask_vis, cv::COLOR_GRAY2BGR);
            cv::hconcat(frame, mask_vis, frame);
          }
        }
      }

      // 发布调试图像
      if (debug_drawing_) {
        cv_bridge::CvImage debug_img;
        debug_img.header = msg->header;
        debug_img.encoding = "bgr8";
        debug_img.image = frame;
        debug_img_pub_.publish(debug_img.toImageMsg());
        cv::imshow("Yellow Detection", frame);
        cv::waitKey(1);
      }

    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
  }

  // 成员变量
  image_transport::Subscriber image_sub_;
  image_transport::Publisher debug_img_pub_; 
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  cv::Mat camera_matrix_, dist_coeffs_;
  bool camera_info_received_;
  bool debug_drawing_ = true;
  
  // TF2相关变量
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContourPoseNode>());
  rclcpp::shutdown();
  return 0;
}
