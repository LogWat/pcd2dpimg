#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

#include <Eigen/Core>

rmw_qos_profile_t qos_profile_image {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_image = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_image.history,
        qos_profile_image.depth
    ),
    qos_profile_image);

rmw_qos_profile_t qos_profile_lidar{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_lidar = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_lidar.history,
        qos_profile_lidar.depth
    ),
    qos_profile_lidar);

namespace pcd2dpimg {
class Pcd2DpImg : public rclcpp::Node
{
public:
    Pcd2DpImg(const rclcpp::NodeOptions &options);
    Pcd2DpImg(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:
    void cb_caminfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void cb_lidar_and_image(
        const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg,
        const sensor_msgs::msg::Image::SharedPtr image_msg
    );

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dp_img_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::Image
    >;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;

    bool get_caminfo_;
    // sub, sub, sub, pub
    std::string lidar_topic_, image_topic_, caminfo_topic_, output_topic_;
    std::string lidar_frame_id_;

    // カメラ関係
    Eigen::Matrix3f K_;                         // カメラ内部パラメータ行列
    uint32_t width_, height_;                    // 画像サイズ
    tf2::Stamped<tf2::Transform> tf_lidar2cam_; // LiDARからカメラへの変換行列
    Eigen::Affine3f lidar2cam_;                 // LiDARからカメラへの変換行列

    // tf関係
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

} // namespace pcd2dpimg