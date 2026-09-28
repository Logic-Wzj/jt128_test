// JT128 真实雷达 -> Gazebo 仿真 HIL 中继
//
// 做三件事（否则仿真 nav2 用不了真实点云）：
//   1. frame_id: front_jt128 -> front_mid360（仿真 TF 树里存在的那颗雷达帧）
//   2. header.stamp: 真实绝对时间 -> 仿真时钟（仿真 use_sim_time=true，
//      时间戳不对 costmap 做 TF 变换时会报 extrapolation 直接丢帧）
//   3. 可选抽稀：stride / max_points，真雷达 115200~230400 点/帧，costmap 吃不消时可降
//
// 参数：
//   in_topic       默认 /lidar_points
//   out_topic      默认 /jt128/points
//   target_frame   默认 front_mid360
//   stamp_offset_ms 时间戳回退量（整数毫秒），默认 100。**HIL 必须项**：中继用 now() 打戳时，
//                  点云会比 Gazebo 发布的 TF 快十几毫秒，nav2 的 TF 过滤器会判
//                  "Lookup would require extrapolation into the future" 而丢掉整帧。
//   stride         默认 1（每 N 点留 1）
//   max_points     默认 0（不限）
//   qos_reliable   默认 false（传感器流用 best-effort；大点云配 RELIABLE 会被慢订阅者反压）
//   use_sim_time   标准 ROS 参数，仿真时必须 true

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using sensor_msgs::msg::PointCloud2;

class Jt128SimRelay : public rclcpp::Node
{
public:
  Jt128SimRelay() : rclcpp::Node("jt128_sim_relay")
  {
    in_topic_ = declare_parameter<std::string>("in_topic", "/lidar_points");
    out_topic_ = declare_parameter<std::string>("out_topic", "/jt128/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "front_mid360");
    stride_ = std::max<int64_t>(1, declare_parameter<int64_t>("stride", 1));
    max_points_ = std::max<int64_t>(0, declare_parameter<int64_t>("max_points", 0));
    qos_reliable_ = declare_parameter<bool>("qos_reliable", false);
    stamp_offset_ms_ = declare_parameter<int64_t>("stamp_offset_ms", 100);
    log_interval_ = declare_parameter<double>("log_interval_sec", 2.0);

    // 订阅：兼容真雷达驱动的 reliable 发布
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (qos_reliable_) {
      pub_qos.reliable();
    } else {
      pub_qos.best_effort();
    }

    pub_ = create_publisher<PointCloud2>(out_topic_, pub_qos);
    sub_ = create_subscription<PointCloud2>(
      in_topic_, sub_qos,
      std::bind(&Jt128SimRelay::onCloud, this, std::placeholders::_1));

    t_last_log_ = now();
    RCLCPP_INFO(get_logger(),
      "HIL 中继：%s -> %s | frame -> %s | stride=%ld max_points=%ld | qos=%s | 时间戳回退 %ld ms",
      in_topic_.c_str(), out_topic_.c_str(), target_frame_.c_str(),
      static_cast<long>(stride_), static_cast<long>(max_points_),
      qos_reliable_ ? "reliable" : "best_effort", static_cast<long>(stamp_offset_ms_));
  }

private:
  void onCloud(const PointCloud2::SharedPtr msg)
  {
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    if (n == 0 || msg->point_step == 0) {
      return;
    }

    size_t step = static_cast<size_t>(stride_);
    if (max_points_ > 0 && n > static_cast<size_t>(max_points_)) {
      const size_t need = (n + static_cast<size_t>(max_points_) - 1) /
                          static_cast<size_t>(max_points_);
      step = std::max(step, need);
    }

    out_ = *msg;                       // 数据一起带过来（一次 memcpy，微秒级）
    out_.header.frame_id = target_frame_;
    // 回退一点时间戳，保证该时刻的 TF 已经在缓冲里（否则 nav2 判"外推到未来"丢帧）
    const auto stamp = now() - rclcpp::Duration::from_seconds(static_cast<double>(stamp_offset_ms_) / 1000.0);
    out_.header.stamp = stamp;         // use_sim_time=true 时即为仿真时钟

    if (step == 1) {
      out_.width = static_cast<uint32_t>(n);
    } else {
      const size_t keep = (n + step - 1) / step;
      const size_t ps = msg->point_step;
      out_.data.resize(keep * ps);
      for (size_t i = 0, j = 0; i < n; i += step, ++j) {
        std::memcpy(out_.data.data() + j * ps, msg->data.data() + i * ps, ps);
      }
      out_.width = static_cast<uint32_t>(keep);
    }
    out_.height = 1;
    out_.row_step = out_.width * out_.point_step;

    pub_->publish(out_);

    ++n_frames_;
    pts_out_ += out_.width;
    const auto t = now();
    const double dt = (t - t_last_log_).seconds();
    if (dt >= log_interval_) {
      RCLCPP_INFO(get_logger(),
        "%.1f 帧/s | 入 %zu 点 -> 出 %u 点 | 累计 %lu 帧 / %.1f M 点",
        (n_frames_ - n_at_log_) / dt, n, out_.width,
        static_cast<unsigned long>(n_frames_), pts_out_ / 1e6);
      t_last_log_ = t;
      n_at_log_ = n_frames_;
    }
  }

  std::string in_topic_, out_topic_, target_frame_;
  int64_t stride_{1}, max_points_{0};
  int64_t stamp_offset_ms_{100};
  bool qos_reliable_{false};
  double log_interval_{2.0};

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  PointCloud2 out_;
  uint64_t n_frames_{0}, n_at_log_{0}, pts_out_{0};
  rclcpp::Time t_last_log_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jt128SimRelay>());
  rclcpp::shutdown();
  return 0;
}
