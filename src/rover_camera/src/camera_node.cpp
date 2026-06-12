#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <sys/mman.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace libcamera;

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
        pub_info_  = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/rover/camera/camera_info", rclcpp::SensorDataQoS());

        if (!use_sim_) {
            if (!open_camera()) {
                RCLCPP_FATAL(get_logger(), "Failed to open camera");
                throw std::runtime_error("camera open failed");
            }
        }

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / fps_),
            std::bind(&CameraNode::publish_frame, this));
    }

    ~CameraNode()
    {
        running_ = false;
        timer_.reset();
        if (camera_) {
            camera_->requestCompleted.disconnect(this);
            camera_->stop();
        }
        requests_.clear();
        if (allocator_ && stream_) {
            allocator_->free(stream_);
        }
        if (camera_) {
            camera_->release();
            camera_.reset();
        }
        if (cm_) {
            cm_->stop();
        }
    }

private:
    bool open_camera()
    {
        cm_ = std::make_unique<CameraManager>();
        if (cm_->start() != 0) {
            RCLCPP_ERROR(get_logger(), "CameraManager::start() failed");
            return false;
        }
        if (cm_->cameras().empty()) {
            RCLCPP_ERROR(get_logger(), "No cameras detected");
            return false;
        }

        camera_ = cm_->cameras()[0];
        if (camera_->acquire() != 0) {
            RCLCPP_ERROR(get_logger(), "Camera::acquire() failed — in use by another process?");
            return false;
        }

        // Request Viewfinder role (ISP-processed output, suitable for video)
        config_ = camera_->generateConfiguration({StreamRole::Viewfinder});
        StreamConfiguration &scfg = config_->at(0);
        scfg.pixelFormat = formats::BGR888;
        scfg.size        = {static_cast<unsigned int>(width_),
                            static_cast<unsigned int>(height_)};
        scfg.bufferCount = 4;

        auto status = config_->validate();
        if (status == CameraConfiguration::Invalid) {
            RCLCPP_ERROR(get_logger(), "Camera configuration rejected");
            return false;
        }
        // validate() may adjust format/size — store the actual values
        width_  = static_cast<int>(scfg.size.width);
        height_ = static_cast<int>(scfg.size.height);
        pixel_format_ = scfg.pixelFormat;
        RCLCPP_INFO(get_logger(), "Camera config: %s %dx%d",
            pixel_format_.toString().c_str(), width_, height_);

        if (camera_->configure(config_.get()) != 0) {
            RCLCPP_ERROR(get_logger(), "Camera::configure() failed");
            return false;
        }
        stream_ = scfg.stream();

        allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_) < 0) {
            RCLCPP_ERROR(get_logger(), "FrameBufferAllocator::allocate() failed");
            return false;
        }

        for (const auto &buf : allocator_->buffers(stream_)) {
            auto req = camera_->createRequest();
            if (!req || req->addBuffer(stream_, buf.get()) != 0) {
                RCLCPP_ERROR(get_logger(), "Failed to create or populate request");
                return false;
            }
            requests_.push_back(std::move(req));
        }

        // Callback fires from libcamera's internal thread — mutex protects latest_frame_
        camera_->requestCompleted.connect(this, [this](Request *req) {
            on_request_completed(req);
        });

        ControlList controls(camera_->controls());
        controls.set(controls::AeEnable, false);
        controls.set(controls::ExposureTime, 8000);      // 8 ms
        controls.set(controls::AnalogueGain, 8.0f);

        if (camera_->start(&controls) != 0) {
            RCLCPP_ERROR(get_logger(), "Camera::start() failed");
            return false;
        }

        for (auto &req : requests_) {
            if (camera_->queueRequest(req.get()) != 0) {
                RCLCPP_ERROR(get_logger(), "Camera::queueRequest() failed");
                return false;
            }
        }

        RCLCPP_INFO(get_logger(), "Camera ready: %dx%d @ %.0f fps, exposure 8 ms",
            width_, height_, fps_);
        return true;
    }

    void on_request_completed(Request *request)
    {
        if (!running_ || request->status() == Request::RequestCancelled) {
            return;
        }

        const FrameBuffer *buf = request->buffers().at(stream_);
        const FrameBuffer::Plane &plane = buf->planes()[0];

        void *mem = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED,
                         plane.fd.get(), plane.offset);
        if (mem != MAP_FAILED) {
            // BGR888 → RGB so ORB-SLAM3 receives "rgb8" encoding
            cv::Mat raw(height_, width_, CV_8UC3, mem);
            cv::Mat rgb;
            cv::cvtColor(raw, rgb, cv::COLOR_BGR2RGB);
            {
                std::lock_guard<std::mutex> lk(frame_mutex_);
                latest_frame_ = std::move(rgb);
            }
            munmap(mem, plane.length);
        }

        // Recycle buffer for next capture
        request->reuse(Request::ReuseBuffers);
        if (running_) {
            camera_->queueRequest(request);
        }
    }

    void publish_frame()
    {
        cv::Mat frame;
        if (use_sim_) {
            frame = make_checkerboard();
        } else {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            if (latest_frame_.empty()) return;
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
        info.header = img.header;
        info.width  = img.width;
        info.height = img.height;
        pub_info_->publish(info);
    }

    cv::Mat make_checkerboard()
    {
        constexpr int sq = 40;
        cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int y = 0; y < height_; ++y)
            for (int x = 0; x < width_; ++x)
                if (((x / sq) + (y / sq)) % 2 == 0)
                    img.at<cv::Vec3b>(y, x) = {200, 200, 200};
        return img;
    }

    bool use_sim_;
    int width_, height_;
    double fps_;
    std::string frame_id_;
    PixelFormat pixel_format_;

    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> config_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    Stream *stream_{nullptr};
    std::vector<std::unique_ptr<Request>> requests_;

    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    std::atomic<bool> running_{true};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
