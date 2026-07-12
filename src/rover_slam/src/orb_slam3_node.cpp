#include <deque>
#include <memory>
#include <string>
#include <csignal>
#include <cstdio>
#include <execinfo.h>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <System.h>

static const char * state_name(int s)
{
    switch (s) {
        case -1: return "NOT_READY";
        case  0: return "NO_IMAGES";
        case  1: return "NOT_INIT";
        case  2: return "OK";
        case  3: return "RECENTLY_LOST";
        case  4: return "LOST";
        default: return "UNKNOWN";
    }
}

static void sigsegv_handler(int)
{
    void * frames[32];
    int n = backtrace(frames, 32);
    fprintf(stderr, "\n[orb_slam3_node] SIGSEGV — stack trace:\n");
    backtrace_symbols_fd(frames, n, STDERR_FILENO);
    std::signal(SIGSEGV, SIG_DFL);
    std::raise(SIGSEGV);
}

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

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
            vocab, settings, ORB_SLAM3::System::RGBD, /*use_viewer=*/false);

        sub_rgb_.subscribe(this, "/rover/camera/image_raw", rmw_qos_profile_sensor_data);
        sub_depth_.subscribe(this, "/rover/camera/depth",    rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), sub_rgb_, sub_depth_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.6));
        sync_->registerCallback(&OrbSlam3Node::on_images, this);

        pub_pose_  = create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/pose", 10);
        pub_state_ = create_publisher<std_msgs::msg::Int32>("/orb_slam3/state", 10);
        pub_debug_ = create_publisher<sensor_msgs::msg::Image>(
            "/orb_slam3/debug_image", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "ORB-SLAM3 RGBD node ready");
    }

    ~OrbSlam3Node()
    {
        if (slam_) {
            sub_rgb_.unsubscribe();
            sub_depth_.unsubscribe();
            slam_->Shutdown();
        }
    }

private:
    void on_images(
        const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
    {
        cv_bridge::CvImageConstPtr rgb_ptr;
        cv_bridge::CvImageConstPtr depth_ptr;
        try {
            rgb_ptr   = cv_bridge::toCvShare(rgb_msg,   "rgb8");
            depth_ptr = cv_bridge::toCvShare(depth_msg, "32FC1");
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        recent_frames_.push_back(rgb_ptr->image.clone());
        if (recent_frames_.size() > kFrameBuffer) {
            recent_frames_.pop_front();
        }

        const double ts = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;
        Sophus::SE3f pose;
        try {
            pose = slam_->TrackRGBD(rgb_ptr->image, depth_ptr->image, ts);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "TrackRGBD threw: %s", e.what());
            return;
        }

        const int cur_state    = slam_->GetTrackingState();
        const auto tracked_kps = slam_->GetTrackedKeyPointsUn();
        ++frame_count_;

        if (frame_count_ % 75 == 0) {
            long rss_kb = 0;
            if (FILE * f = std::fopen("/proc/self/status", "r")) {
                char line[128];
                while (std::fgets(line, sizeof(line), f)) {
                    if (std::sscanf(line, "VmRSS: %ld kB", &rss_kb) == 1) break;
                }
                std::fclose(f);
            }
            RCLCPP_INFO(get_logger(), "SLAM: state=%s  kps=%zu  rss=%ld MB",
                state_name(cur_state), tracked_kps.size(), rss_kb / 1024);
            std_msgs::msg::Int32 s; s.data = cur_state;
            pub_state_->publish(s);
        }

        if (cur_state != prev_state_) {
            RCLCPP_WARN(get_logger(),
                "SLAM state  %s → %s   kps=%zu  frame=%lu",
                state_name(prev_state_), state_name(cur_state),
                tracked_kps.size(), frame_count_);
            std_msgs::msg::Int32 s; s.data = cur_state;
            pub_state_->publish(s);

            if (cur_state == 3 || cur_state == 4) {
                int idx = 0;
                for (const auto & frame : recent_frames_) {
                    cv::Mat bgr;
                    cv::cvtColor(frame, bgr, cv::COLOR_RGB2BGR);
                    const std::string path = "/tmp/slam_loss_" +
                        std::to_string(loss_count_) + "_f" +
                        std::to_string(idx++) + ".jpg";
                    cv::imwrite(path, bgr);
                }
                RCLCPP_WARN(get_logger(),
                    "Saved %zu frames → /tmp/slam_loss_%d_f*.jpg",
                    recent_frames_.size(), loss_count_++);
            }

            prev_state_ = cur_state;
        }

        if (pub_debug_->get_subscription_count() > 0) {
            publish_debug(rgb_ptr->image, rgb_msg->header, cur_state, tracked_kps);
        }

        if (cur_state != 2) {
            return;
        }

        geometry_msgs::msg::PoseStamped out;
        out.header          = rgb_msg->header;
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

        pub_pose_->publish(out);
    }

    void publish_debug(
        const cv::Mat & rgb,
        const std_msgs::msg::Header & header,
        int state,
        const std::vector<cv::KeyPoint> & kps)
    {
        cv::Mat vis;
        cv::cvtColor(rgb, vis, cv::COLOR_RGB2BGR);

        const cv::Scalar dot_color = (state == 2)
            ? cv::Scalar(0, 220, 0)
            : cv::Scalar(0, 0, 220);
        for (const auto & kp : kps) {
            cv::circle(vis, kp.pt, 3, dot_color, -1);
        }

        const std::string label = std::string(state_name(state)) +
            "  kps=" + std::to_string(kps.size());
        cv::putText(vis, label, cv::Point(4, 18),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

        sensor_msgs::msg::Image out;
        out.header   = header;
        out.height   = static_cast<uint32_t>(vis.rows);
        out.width    = static_cast<uint32_t>(vis.cols);
        out.encoding = "bgr8";
        out.step     = static_cast<uint32_t>(vis.cols * 3);
        out.data.assign(vis.data, vis.data + vis.total() * 3);
        pub_debug_->publish(out);
    }

    static constexpr size_t kFrameBuffer = 5;

    std::unique_ptr<ORB_SLAM3::System> slam_;

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_rgb_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_;
    std::shared_ptr<Synchronizer> sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_state_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;

    int      prev_state_{-1};
    uint64_t frame_count_{0};
    int      loss_count_{0};
    std::deque<cv::Mat> recent_frames_;
};

int main(int argc, char ** argv)
{
    std::signal(SIGSEGV, sigsegv_handler);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrbSlam3Node>());
    rclcpp::shutdown();
    return 0;
}
