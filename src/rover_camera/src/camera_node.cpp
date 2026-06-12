#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        declare_parameter("use_sim", false);
        declare_parameter("frame_width", 320);
        declare_parameter("frame_height", 240);
        declare_parameter("publish_rate_hz", 15.0);
        declare_parameter("frame_id", std::string("camera_link"));

        use_sim_  = get_parameter("use_sim").as_bool();
        width_    = get_parameter("frame_width").as_int();
        height_   = get_parameter("frame_height").as_int();
        fps_      = get_parameter("publish_rate_hz").as_double();
        frame_id_ = get_parameter("frame_id").as_string();

        pub_image_ = create_publisher<sensor_msgs::msg::Image>(
            "/rover/camera/image_raw", rclcpp::SensorDataQoS());
        pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/rover/camera/camera_info", rclcpp::SensorDataQoS());

        if (!use_sim_) {
            open_camera();
            // Background thread: cap_.read() blocks on I/O — no GIL equivalent in C++,
            // so the ROS2 timer below fires freely at the full 15 Hz target.
            capture_thread_ = std::thread(&CameraNode::capture_loop, this);
        }

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / fps_),
            std::bind(&CameraNode::publish_frame, this));
    }

    ~CameraNode()
    {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    void open_camera()
    {
        // Try GStreamer + libcamerasrc first (best frame-rate control on Pi).
        // exposure-time in µs (8000 = 8 ms), analogue-gain compensates brightness.
        // drop=true keeps only the newest frame so we never publish stale data.
        const std::string gst =
            "libcamerasrc ae-enable=false exposure-time=8000 analogue-gain=8 ! "
            "video/x-raw,width=" + std::to_string(width_) +
            ",height=" + std::to_string(height_) +
            ",framerate=" + std::to_string(static_cast<int>(fps_)) + "/1 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=2 sync=false";

        cap_.open(gst, cv::CAP_GSTREAMER);
        if (cap_.isOpened()) {
            RCLCPP_INFO(get_logger(),
                "Camera ready (GStreamer) %dx%d @ %.0f fps, exposure 8ms", width_, height_, fps_);
            return;
        }

        // GStreamer unavailable — fall back to V4L2.
        RCLCPP_WARN(get_logger(), "GStreamer failed, trying V4L2 /dev/video0");
        cap_.open(0, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS,          fps_);
        if (cap_.isOpened()) {
            RCLCPP_INFO(get_logger(),
                "Camera ready (V4L2) %dx%d @ %.0f fps", width_, height_, fps_);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to open camera via GStreamer or V4L2");
        }
    }

    void capture_loop()
    {
        while (running_ && rclcpp::ok()) {
            if (!cap_.isOpened()) {
                break;
            }
            cv::Mat frame;
            if (cap_.read(frame) && !frame.empty()) {
                // Pi Camera via GStreamer gives BGR; convert to RGB so downstream
                // nodes (ORB-SLAM3) receive the encoding they declare ("rgb8").
                cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
                std::lock_guard<std::mutex> lk(frame_mutex_);
                latest_frame_ = frame;  // shallow ref-count copy, no pixel copy
            }
        }
    }

    void publish_frame()
    {
        cv::Mat frame;
        if (use_sim_) {
            frame = make_checkerboard();
        } else {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            if (latest_frame_.empty()) {
                return;
            }
            frame = latest_frame_.clone();
        }

        auto stamp = get_clock()->now();

        sensor_msgs::msg::Image img;
        img.header.stamp    = stamp;
        img.header.frame_id = frame_id_;
        img.height          = static_cast<uint32_t>(frame.rows);
        img.width           = static_cast<uint32_t>(frame.cols);
        img.encoding        = "rgb8";
        img.is_bigendian    = false;
        img.step            = static_cast<uint32_t>(frame.cols * 3);
        img.data.assign(frame.data, frame.data + frame.total() * 3);
        pub_image_->publish(img);

        sensor_msgs::msg::CameraInfo info;
        info.header.stamp    = stamp;
        info.header.frame_id = frame_id_;
        info.width           = static_cast<uint32_t>(frame.cols);
        info.height          = static_cast<uint32_t>(frame.rows);
        pub_info_->publish(info);
    }

    cv::Mat make_checkerboard()
    {
        constexpr int sq = 40;
        cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (((x / sq) + (y / sq)) % 2 == 0) {
                    img.at<cv::Vec3b>(y, x) = {200, 200, 200};
                }
            }
        }
        return img;
    }

    bool use_sim_;
    int width_, height_;
    double fps_;
    std::string frame_id_;

    cv::VideoCapture cap_;
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    std::thread capture_thread_;
    std::atomic<bool> running_{true};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
