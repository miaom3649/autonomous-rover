#include <deque>
#include <memory>
#include <string>
#include <csignal>
#include <cstdio>
#include <execinfo.h>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
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

        pub_pose_  = create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/pose", 10);
        pub_debug_ = create_publisher<sensor_msgs::msg::Image>(
            "/orb_slam3/debug_image", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "ORB-SLAM3 monocular node ready");
    }

    ~OrbSlam3Node()
    {
        if (slam_) {
            sub_.reset();
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

        // Rolling buffer of last N frames for post-failure inspection
        recent_frames_.push_back(cv_ptr->image.clone());
        if (recent_frames_.size() > kFrameBuffer) {
            recent_frames_.pop_front();
        }

        const double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        Sophus::SE3f pose;
        try {
            pose = slam_->TrackMonocular(cv_ptr->image, ts);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "TrackMonocular threw: %s", e.what());
            return;
        }

        const int cur_state    = slam_->GetTrackingState();
        const auto tracked_kps = slam_->GetTrackedKeyPointsUn();
        ++frame_count_;

        // Heartbeat every 5 s (75 frames @ 15 fps)
        if (frame_count_ % 75 == 0) {
            RCLCPP_INFO(get_logger(), "SLAM: state=%s  kps=%zu",
                state_name(cur_state), tracked_kps.size());
        }

        // State transition — log with keypoint count at the moment of change
        if (cur_state != prev_state_) {
            RCLCPP_WARN(get_logger(),
                "SLAM state  %s → %s   kps=%zu  frame=%lu",
                state_name(prev_state_), state_name(cur_state),
                tracked_kps.size(), frame_count_);

            // On tracking loss: save recent frames to /tmp/ for inspection
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

        // Debug image — only render when a subscriber is connected
        if (pub_debug_->get_subscription_count() > 0) {
            publish_debug(cv_ptr->image, msg->header, cur_state, tracked_kps);
        }

        if (cur_state != 2) {
            return;
        }

        geometry_msgs::msg::PoseStamped out;
        out.header          = msg->header;
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
            ? cv::Scalar(0, 220, 0)    // green — tracking OK
            : cv::Scalar(0, 0, 220);   // red   — lost
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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
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
