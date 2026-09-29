// JT128 真实雷达 -> Gazebo 仿真 HIL 中继
//
// 为什么需要这个节点（三件事，只有"转发者"能做）：
//   1. frame_id: front_jt128 -> front_mid360
//      真雷达发的帧名是 front_jt128，而仿真的 TF 树里没有这个帧（只有 front_mid360），
//      costmap 做 TF 变换时找不到父链 → 整帧被丢。
//      注意：改帧名不能靠 remap（remap 只改话题名，改不了消息里的 header.frame_id）；
//      加一条假静态 TF 也能解决帧名，但解决不了下面第 2 条，所以统一在这里做。
//   2. header.stamp: 雷达自己的时钟 -> 仿真时钟
//      实测真雷达 header.stamp ≈ 1029 s（上电累计秒），而仿真时钟只有几十秒。
//      驱动的两个选项（use_timestamp_type: 0=点云时间戳 / 1=接收时间戳）都不是 ROS/sim 时间——
//      驱动是从帧时间戳直接填 header（manager/source_driver_ros2.hpp:313-314），
//      所以仿真里**没有别的办法**能让戳跟随 /clock，只能由这里用 now() 重打戳。
//      再回退 stamp_offset_ms：中继比 Gazebo 广播 TF 快十几毫秒，不回退 nav2 会判
//      "Lookup would require extrapolation into the future"，把整帧丢掉。
//   3. 可选抽稀：真雷达出厂双回波 230400 点/帧（5.99 MB），costmap/RViz 会吃力，
//      用 stride（每 N 点留 1）或 max_points（抽到不超过该点数）降载。
//
// 实测（2026-09-28，JT128 双回波 10 Hz）：/lidar_points -> /jt128/points 稳定 9.99 Hz、零丢帧。
//
// 参数：
//   in_topic       默认 /lidar_points
//   out_topic      默认 /jt128/points
//   target_frame   默认 front_mid360
//   stamp_offset_ms 时间戳回退量（整数毫秒），默认 100。**HIL 必须项**：中继用 now() 打戳时，
//                  点云会比 Gazebo 发布的 TF 快十几毫秒，nav2 的 TF 过滤器会判
//                  "外推到未来"而丢掉整帧。
//   stride         默认 1（每 N 点留 1）
//   max_points     默认 0（不限）
//   qos_reliable   默认 false（传感器流用 best-effort；大点云配 RELIABLE 会被慢订阅者反压）
//   log_interval_sec 默认 2.0（打印帧率/点数的间隔）
//   use_sim_time   标准 ROS 参数，仿真时必须 true（否则 now() 取的是墙钟，戳又不对了）

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
    // ---- 参数（全部可用 launch 参数或 -p 覆盖）----
    in_topic_ = declare_parameter<std::string>("in_topic", "/lidar_points");
    out_topic_ = declare_parameter<std::string>("out_topic", "/jt128/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "front_mid360");
    // stride/max_points 都做下界保护：负数会被"当 0 用"从而出错，这里直接夹到合法范围
    stride_ = std::max<int64_t>(1, declare_parameter<int64_t>("stride", 1));
    max_points_ = std::max<int64_t>(0, declare_parameter<int64_t>("max_points", 0));
    qos_reliable_ = declare_parameter<bool>("qos_reliable", false);
    stamp_offset_ms_ = declare_parameter<int64_t>("stamp_offset_ms", 100);
    log_interval_ = declare_parameter<double>("log_interval_sec", 2.0);

    // ---- QoS ----
    // 订阅用 best_effort + depth 5：best_effort 能同时兼容"reliable 发布者"和"best_effort 发布者"；
    // depth 5 是为了容忍偶发的处理抖动，不至于把刚到的一帧直接挤掉。
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    // 发布默认 best_effort + depth 1：点云是"只有最新一帧有意义"的数据，
    // 用 reliable 会在下游变慢时堆积重传、把延迟越拉越大（要降载应该用 stride，而不是重传）。
    // 需要人和工具（ros2 topic echo / RViz）稳定看到时，可 qos_reliable:=true 临时切换。
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

    // 启动时把实际生效的配置打出来，方便和 launch/命令行参数对照排查
    t_last_log_ = now();
    RCLCPP_INFO(get_logger(),
      "HIL 中继：%s -> %s | frame -> %s | stride=%ld max_points=%ld | qos=%s | 时间戳回退 %ld ms",
      in_topic_.c_str(), out_topic_.c_str(), target_frame_.c_str(),
      static_cast<long>(stride_), static_cast<long>(max_points_),
      qos_reliable_ ? "reliable" : "best_effort", static_cast<long>(stamp_offset_ms_));
  }

private:
  // 每来一帧点云调用一次。整个函数只做"改帧名 + 重打时间戳 + 可选抽稀"，不碰点的内容。
  void onCloud(const PointCloud2::SharedPtr msg)
  {
    // 点数：PointCloud2 通常是 width=N / height=1，但规范允许二维，所以两者相乘
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    // 空帧或字段步长为 0（消息不合法）直接忽略，避免后面除零/越界
    if (n == 0 || msg->point_step == 0) {
      return;
    }

    // ---- 计算抽稀步长 ----
    // 两个参数同时生效时取"更狠"的那个：
    //   stride=N      → 每 N 点留 1
    //   max_points=M  → 抽到不超过 M 点，需要的步长是 ceil(n/M)
    size_t step = static_cast<size_t>(stride_);
    if (max_points_ > 0 && n > static_cast<size_t>(max_points_)) {
      const size_t need = (n + static_cast<size_t>(max_points_) - 1) /
                          static_cast<size_t>(max_points_);
      step = std::max(step, need);
    }

    // 整条消息（header + fields + data）先整体拷过来，之后只改需要改的字段。
    // 这是一次 ~6 MB 的 memcpy（微秒级），比"逐个字段手工拼一条新消息"简单且不易错。
    out_ = *msg;
    out_.header.frame_id = target_frame_;
    // 用 now() 重打戳：use_sim_time=true 时 now() 就是仿真时钟，天然和 Gazebo 的 TF 对齐。
    // 再往回退一点，确保该时刻的 TF 已经进了 nav2 的缓冲区（否则判"外推到未来"丢帧）。
    const auto stamp = now() - rclcpp::Duration::from_seconds(static_cast<double>(stamp_offset_ms_) / 1000.0);
    out_.header.stamp = stamp;

    if (step == 1) {
      // 不抽稀：点数不变
      out_.width = static_cast<uint32_t>(n);
    } else {
      // 抽稀：按点重排。keep = ceil(n/step) 是抽完的点数。
      // 顺序保留（不是随机抽），这样方位角分布仍然均匀，costmap 标记不会出现空洞。
      const size_t keep = (n + step - 1) / step;
      const size_t ps = msg->point_step;
      out_.data.resize(keep * ps);
      // 逐点 memcpy ps 字节：源按 i*ps 步进，目的按 j*ps 连续写
      for (size_t i = 0, j = 0; i < n; i += step, ++j) {
        std::memcpy(out_.data.data() + j * ps, msg->data.data() + i * ps, ps);
      }
      out_.width = static_cast<uint32_t>(keep);
    }
    // 这几个字段必须和抽稀后的实际数据保持一致，否则订阅端会按 row_step 解析出错位
    out_.height = 1;
    out_.row_step = out_.width * out_.point_step;

    pub_->publish(out_);

    // ---- 统计与限频日志 ----
    // 每 log_interval_ 秒打一行"帧率 | 入 N 点 -> 出 M 点 | 累计"，这是判断
    // "中继有没有在干活、抽了多少"的第一手证据（排查 HIL 时先看这一行）。
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

  // ---- 配置 ----
  std::string in_topic_, out_topic_, target_frame_;
  int64_t stride_{1}, max_points_{0};
  int64_t stamp_offset_ms_{100};
  bool qos_reliable_{false};
  double log_interval_{2.0};

  // ---- 运行期对象与统计 ----
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  PointCloud2 out_;                  // 复用的输出消息，避免每帧重新分配
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
