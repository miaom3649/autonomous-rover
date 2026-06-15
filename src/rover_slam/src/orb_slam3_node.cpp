#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <System.h>

class OrbSlam3Node : public rclcpp::Node
{
public:
    OrbSlam3Node() : Node("orb_slam3_node")
    {
        declare_parameter("vocab_path", "");
        declare_parameter("settings_path", "");

        const auto vocab    = get_parameter("vocab_path").as_string();
        const auto settings = get_parameter("settings_path").as_string();

        if (vocab.empty() || settings.empty()) {
            RCLCPP_FATAL(get_logger(), "vocab_path and settings_path are required parameters");
            throw std::runtime_error("missing required parameters");
        }

        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocab, settings, ORB_SLAM3::System::MONOCULAR, /*use_viewer=*/false);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/rover/camera/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&OrbSlam3Node::on_image, this, std::placeholders::_1));

        pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/pose", 10);

        RCLCPP_INFO(get_logger(), "ORB-SLAM3 monocular node ready");
    }

    ~OrbSlam3Node()
    {
        if (slam_) {
            slam_->Shutdown();
        }
    }

private:
    void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        const double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        Sophus::SE3f pose;
        try {
            pose = slam_->TrackMonocular(cv_ptr->image, ts);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "TrackMonocular threw: %s", e.what());
            return;
        }

        // Tracking::OK == 2
        if (slam_->GetTrackingState() != 2) {
            return;
        }

        geometry_msgs::msg::PoseStamped out;
        out.header        = msg->header;
        out.header.frame_id = "map";

        const auto t = pose.translation();
        const auto q = pose.unit_quaternion();
        out.pose.position.x    = t.x();
        out.pose.position.y    = t.y();
        out.pose.position.z    = t.z();
        out.pose.orientation.x = q.x();
        out.pose.orientation.y = q.y();
        out.pose.orientation.z = q.z();
        out.pose.orientation.w = q.w();

        pub_->publish(out);
    }

    std::unique_ptr<ORB_SLAM3::System> slam_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrbSlam3Node>());
    rclcpp::shutdown();
    return 0;
}
