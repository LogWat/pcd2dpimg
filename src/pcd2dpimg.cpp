#include "pcd2dpimg/pcd2dpimg.hpp"
#include <rclcpp/rclcpp.hpp>

namespace pcd2dpimg {
Pcd2DpImg::Pcd2DpImg(const rclcpp::NodeOptions &options)
    : Node("pcd2dpimg", options), get_caminfo_(false)
{
    this->declare_parameter<std::string>("lidar_topic", "lidar");              // 3D点群topic
    this->declare_parameter<std::string>("image_topic", "image");              // 画像topic
    this->declare_parameter<std::string>("camera_info_topic", "camera_info");  // カメラ情報topic
    this->declare_parameter<std::string>("lidar_frame_id", "lidar_link");             // フレームID
    this->declare_parameter<std::string>("output_topic", "dpimg");             // 出力topic

    this->get_parameter("lidar_topic", lidar_topic_);
    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("camera_info_topic", caminfo_topic_);
    this->get_parameter("lidar_frame_id", lidar_frame_id_);
    this->get_parameter("output_topic", output_topic_);

    pub_dp_img_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, qos_image);
    sub_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        caminfo_topic_, qos_image,
        std::bind(&Pcd2DpImg::cb_caminfo, this, std::placeholders::_1)
    );
    sub_lidar_.subscribe(this, lidar_topic_, qos_profile_lidar);
    sub_image_.subscribe(this, image_topic_, qos_profile_image);
    sync_ = std::make_shared<Sync>(SyncPolicy(10), sub_lidar_, sub_image_);
    sync_->registerCallback(&Pcd2DpImg::cb_lidar_and_image, this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void Pcd2DpImg::cb_caminfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    K_ << msg->k[0], msg->k[1], msg->k[2],
          msg->k[3], msg->k[4], msg->k[5],
          msg->k[6], msg->k[7], msg->k[8];
    width_ = msg->width;
    height_ = msg->height;
    RCLCPP_INFO(this->get_logger(), "Camera Info: [%d, %d]", width_, height_);
    RCLCPP_INFO(this->get_logger(), "Camera Matrix: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
        msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5], msg->k[6], msg->k[7], msg->k[8]);
    try {
        auto tf_msg = tf_buffer_->lookupTransform(
            lidar_frame_id_, "front_realsense_link", rclcpp::Time(0)
        );
        lidar2cam_.setIdentity();
        auto tr = tf_msg.transform.translation;
        lidar2cam_.translation() << tr.x, tr.y, tr.z;
        Eigen::AngleAxisf yaw(0.0f, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf pitch(M_PI/2, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf roll(-M_PI/2, Eigen::Vector3f::UnitX()); // カメラ座標系とLiDAR座標系の軸の向きの違い (realsense only?)
        Eigen::Quaternionf q = yaw * pitch * roll;
        lidar2cam_.linear() = q.toRotationMatrix();
        // auto rot = tf_msg.transform.rotation;
        // auto q2 = Eigen::Quaternionf(rot.w, rot.x, rot.y, rot.z); // TODO
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return;
    }
    // shutdown
    get_caminfo_ = true;
    sub_caminfo_.reset();
}

void Pcd2DpImg::cb_lidar_and_image(
    const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg,
    const sensor_msgs::msg::Image::SharedPtr image_msg
) {
    if (!get_caminfo_) return;
    // realsenseの画像をcv::Matに変換
    auto img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*lidar_msg, *cloud);
    for (const auto &p : cloud->points) {
        // 3D-LiDARの後方の点群は除外
        if (p.x <= 0) continue;
        Eigen::Vector3f p_cam(p.x, p.y, p.z);
        p_cam = lidar2cam_ * p_cam;             // LiDAR座標系からカメラ座標系へ変換
        Eigen::Vector3f p_img = K_ * p_cam;     // カメラ座標系から画像座標系へ変換
        int u = width_ - p_img.x() / p_img.z();
        int v = p_img.y() / p_img.z();
        // 画角外にある点群は除外
        if (u < 0 || u >= (int)width_ || v < 0 || v >= (int)height_) continue;
        img.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 255);
    }
    // publish
    pub_dp_img_->publish(
        *cv_bridge::CvImage(
            image_msg->header,
            sensor_msgs::image_encodings::BGR8,
            img
        ).toImageMsg()
    );
}

} // namespace pcd2dpimg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2dpimg::Pcd2DpImg)