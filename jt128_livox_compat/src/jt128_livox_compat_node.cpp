// JT128 -> Livox Mid-360 点云格式适配层
//
// 为什么需要它（全部有源码依据）：
//   禾赛 JT128 驱动输出的 PointCloud2 字段是：
//       x,y,z,intensity (float32) + ring (uint16) + timestamp (float64)
//       （src/manager/source_driver_ros2.hpp:274-279）
//   而 small_point_lio 的 livox_pointcloud2 适配器按**字段名**读取：
//       x,y,z (float32) + tag (uint8) + timestamp (double)
//       且只接受 (tag & 0x3F) == 0 的点，时间戳按纳秒解释并 ×1e-9
//       （src/lidar_adapter/livox_pointcloud2.h:25-37）
//   → 禾赛点云**没有 tag 字段**，LIO 直接取不到数据。本节点补上 tag 与 line。
//   → 另外 LIO 的回调里会先 lookupTransform(lidar_frame, "base_link")，失败就静默不发里程计，
//     所以配套的 launch 里还会补一条 base_link->livox_frame 的静态 TF。
//
// 时间戳单位换算（默认 ×1e9：秒 → 纳秒）：
//   small_point_lio 的适配器按**纳秒**解释：livox_pointcloud2.h:37 `* 1e-9`。
//   而实测禾赛驱动最终写进 ROS 消息 timestamp 字段的是**秒**：
//     /lidar_points 实测 min=1029.91309、max=1030.013298，一帧跨度 0.1002 s，
//     与 header.stamp 同量级（若为纳秒应为 ~1.03e12、跨度 1e8）。
//   注意别被 SDK 内部表示误导：udp1_4_parser.h:477 确实算出了纳秒
//   （`sensor_timestamp * kMicrosecondToNanosecondInt`），但那是 SDK 内部 PtInfo；
//   最终 LidarPointXYZIRT.timestamp（lidar_types.h:74，double）是秒量级。
//   所以这里必须换算，用参数 timestamp_scale 控制（默认 1e9，设 1 = 不换算）。
//   这条曾经漏掉过（注释里写着"已经是纳秒，不用换算"），后果是 LIO 帧内时间间隔
//   被缩小 1e9 倍、去畸变静默算错。现在 `./launch.sh compatcheck` 会做回归测试兜底。
//
// 输出默认发到 /livox/lidar、frame 用 livox_frame，于是 small_point_lio /
// cpp_lidar_filter / nav2 全都不用改。
//
// 参数：
//   in_topic          默认 /lidar_points
//   out_topic         默认 /livox/lidar
//   target_frame      默认 livox_frame
//   stamp_offset_ms   默认 0（不动 header 时间戳；>0 表示往前挪，仿真里用得上）
//   timestamp_scale   默认 1e9（逐点 timestamp 乘这个系数：禾赛秒 → 纳秒；设 1 不换算）
//   qos_reliable      默认 false（LIO/滤波器都是 SensorDataQoS=best_effort）
//   log_interval_sec  默认 2.0
//   relay_imu         默认 true（把 /lidar_imu 转成 /livox/imu；LIO 需要 IMU 才会工作）
//   in_imu_topic / out_imu_topic  默认 /lidar_imu -> /livox/imu

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class Jt128LivoxCompat : public rclcpp::Node
{
public:
  Jt128LivoxCompat() : rclcpp::Node("jt128_livox_compat")
  {
    // ---- 参数（launch 里 compat.launch.py 会把它们暴露成同名 launch 参数）----
    in_topic_ = declare_parameter<std::string>("in_topic", "/lidar_points");
    out_topic_ = declare_parameter<std::string>("out_topic", "/livox/lidar");
    target_frame_ = declare_parameter<std::string>("target_frame", "livox_frame");
    stamp_offset_ms_ = declare_parameter<int64_t>("stamp_offset_ms", 0);
    timestamp_scale_ = declare_parameter<double>("timestamp_scale", 1e9);
    qos_reliable_ = declare_parameter<bool>("qos_reliable", false);
    log_interval_ = declare_parameter<double>("log_interval_sec", 2.0);
    relay_imu_ = declare_parameter<bool>("relay_imu", true);
    in_imu_topic_ = declare_parameter<std::string>("in_imu_topic", "/lidar_imu");
    out_imu_topic_ = declare_parameter<std::string>("out_imu_topic", "/livox/imu");

    // 点云 QoS：订阅 best_effort（兼容两类发布者），发布默认 best_effort + depth 1
    // （最新一帧才有意义；要给人和工具稳定看就 qos_reliable:=true）
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (qos_reliable_) {
      pub_qos.reliable();
    } else {
      pub_qos.best_effort();
    }

    pub_ = create_publisher<PointCloud2>(out_topic_, pub_qos);
    if (relay_imu_) {
      // IMU 频率高（JT128 实测约 400 Hz），depth 用 50 而不是 1：
      // LIO 靠 IMU 做预积分，偶发卡顿后不该只留一帧。
      auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
      imu_pub_ = create_publisher<Imu>(out_imu_topic_, imu_qos);
      imu_sub_ = create_subscription<Imu>(
        in_imu_topic_, imu_qos,
        std::bind(&Jt128LivoxCompat::onImu, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "IMU 转发：%s -> %s（frame=%s）",
        in_imu_topic_.c_str(), out_imu_topic_.c_str(), target_frame_.c_str());
    }
    sub_ = create_subscription<PointCloud2>(
      in_topic_, sub_qos,
      std::bind(&Jt128LivoxCompat::onCloud, this, std::placeholders::_1));

    // 启动日志：把实际生效的换算系数和话题打出来（排查时先看这一行）
    t_last_log_ = now();
    RCLCPP_INFO(get_logger(),
      "JT128→Livox 适配：%s -> %s | frame=%s | 补 tag(=0)+line(=ring) | header 时间戳偏移 %ld ms | "
      "逐点时间戳 ×%g | qos=%s",
      in_topic_.c_str(), out_topic_.c_str(), target_frame_.c_str(),
      static_cast<long>(stamp_offset_ms_), timestamp_scale_,
      qos_reliable_ ? "reliable" : "best_effort");
  }

private:
  // 按**字段名**查偏移（不写死偏移量：换驱动版本/换雷达也不会错位）。
  // 找不到返回 -1，调用方据此决定"能不能补/能不能换算"。
  static int findFieldOffset(const std::vector<PointField> & fields, const std::string & name)
  {
    for (const auto & f : fields) {
      if (f.name == name) return static_cast<int>(f.offset);
    }
    return -1;
  }

  // IMU 转发：只改 frame（和可选的时间戳回退），其余原样透传 —— LIO 要的就是原始量测。
  void onImu(const Imu::SharedPtr msg)
  {
    imu_out_ = *msg;
    imu_out_.header.frame_id = target_frame_;
    if (stamp_offset_ms_ != 0) {
      rclcpp::Time t(msg->header.stamp);
      t = t - rclcpp::Duration::from_seconds(static_cast<double>(stamp_offset_ms_) / 1000.0);
      imu_out_.header.stamp = t;
    }
    imu_pub_->publish(imu_out_);
    ++imu_count_;
  }

  // 点云转发：补 tag/line + 改帧 + 可选的时间戳换算。点本身的 x/y/z/intensity 一律不动。
  void onCloud(const PointCloud2::SharedPtr msg)
  {
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    if (n == 0 || msg->point_step == 0) return;

    // ---- 只在第一帧探测一次字段布局 ----
    // 布局探测放在运行期而不是编译期，这样"输入到底是禾赛格式还是已经带 tag 的格式"
    // 都能自适应（带 tag 的就只改帧/话题，不重复追加字段）。
    if (!layout_checked_) {
      ring_off_ = findFieldOffset(msg->fields, "ring");
      tag_off_ = findFieldOffset(msg->fields, "tag");
      ts_off_ = findFieldOffset(msg->fields, "timestamp");
      layout_checked_ = true;
      RCLCPP_INFO(get_logger(),
        "输入点云: point_step=%u 字段数=%zu  ring@%d  tag@%d  timestamp@%d %s | 逐点时间戳 ×%g",
        msg->point_step, msg->fields.size(), ring_off_, tag_off_, ts_off_,
        tag_off_ >= 0 ? "(已有 tag，将直接复用)" : "(无 tag，将追加)", timestamp_scale_);
      // 缺 timestamp：LIO 的去畸变会失效（它按字段名取），必须显式提醒
      if (ts_off_ < 0) {
        RCLCPP_WARN(get_logger(),
          "输入没有 timestamp 字段 —— small_point_lio 的 livox 适配器会取不到它（去畸变会失效）");
      } else if (timestamp_scale_ == 1.0) {
        // 设成 1 只在"输入本来就是纳秒"时才对；禾赛是秒，所以这里警告而不是静默放过
        RCLCPP_WARN(get_logger(),
          "timestamp_scale=1：不做换算。只有当输入 timestamp 已经是纳秒时才对；"
          "禾赛驱动的 PointCloud2 是秒，正确值应是 1e9");
      }
    }

    const size_t in_step = msg->point_step;

    // 已有 tag 的输入：只需改 frame / 话题；否则追加 tag、line 两个字段（point_step 26 → 28）
    const bool need_patch = (tag_off_ < 0);
    const size_t out_step = need_patch ? in_step + 2 : in_step;
    // 逐点时间戳需要换算时也必须走拷贝：零拷贝共享的话改不了别人的数据。
    // 顺带做越界保护（timestamp 字段必须完整落在 point_step 之内）。
    const bool scale_ts = (ts_off_ >= 0) && (timestamp_scale_ != 1.0) &&
                          (static_cast<size_t>(ts_off_) + sizeof(double) <= in_step);

    // ---- 头部 ----
    out_.header = msg->header;
    out_.header.frame_id = target_frame_;
    if (stamp_offset_ms_ != 0) {
      rclcpp::Time t(msg->header.stamp);
      t = t - rclcpp::Duration::from_seconds(static_cast<double>(stamp_offset_ms_) / 1000.0);
      out_.header.stamp = t;
    }

    // ---- 字段表 ----
    out_.fields = msg->fields;
    if (need_patch) {
      // 追加两个 uint8：tag 放在原 point_step 处、line 紧跟其后（所以新 point_step = 旧 + 2）
      PointField f;
      f.datatype = PointField::UINT8;
      f.count = 1;
      f.name = "tag";
      f.offset = static_cast<uint32_t>(in_step);
      out_.fields.push_back(f);
      f.name = "line";
      f.offset = static_cast<uint32_t>(in_step + 1);
      out_.fields.push_back(f);
    }

    // ---- 几何字段必须与最终数据一致 ----
    out_.height = 1;
    out_.width = msg->width;
    out_.point_step = static_cast<uint32_t>(out_step);
    out_.row_step = out_.width * out_.point_step;
    out_.is_dense = msg->is_dense;

    if (!need_patch && !scale_ts) {
      // 两个改造都不需要：直接共享数据，零拷贝（例如输入已经带 tag 且时间戳单位已正确）
      out_.data = msg->data;
    } else {
      // 逐点拷贝，顺路把 tag/line 补上、把 timestamp 换算掉
      out_.data.resize(n * out_step);
      const uint8_t * src = msg->data.data();
      uint8_t * dst = out_.data.data();
      for (size_t i = 0; i < n; ++i) {
        // 先把整点原样搬过去（x/y/z/intensity/ring/timestamp 都在里面）
        std::memcpy(dst, src, in_step);
        if (need_patch) {
          // tag = 0：因为 LIO 的判据是 (tag & 0x3F) == 0，0 一定能被接收
          dst[in_step] = 0;
          // line = ring 的低字节：LIO 不用 line，这里只是让格式和 Livox 对齐（128 线 → 0~127）
          dst[in_step + 1] = (ring_off_ >= 0) ?
            src[static_cast<size_t>(ring_off_)] : 0;
        }
        if (scale_ts) {
          // 禾赛 timestamp 是 little-endian float64 的**秒**；LIO 按纳秒解释，故默认 ×1e9。
          // 就地读取-相乘-写回，不引入中间数组。
          double ts = 0.0;
          std::memcpy(&ts, src + ts_off_, sizeof(double));
          ts *= timestamp_scale_;
          std::memcpy(dst + ts_off_, &ts, sizeof(double));
        }
        src += in_step;
        dst += out_step;
      }
    }

    pub_->publish(out_);

    // ---- 统计与限频日志（帧率 / 累计点数 / 输出 point_step / IMU 帧数）----
    ++frames_;
    pts_ += n;
    const auto t = now();
    const double dt = (t - t_last_log_).seconds();
    if (dt >= log_interval_) {
      RCLCPP_INFO(get_logger(),
        "%.1f 帧/s | 累计 %lu 帧 / %.1f M 点 | 输出 point_step=%zu 字段=%zu | IMU %lu 帧",
        (frames_ - frames_at_log_) / dt, static_cast<unsigned long>(frames_),
        pts_ / 1e6, out_step, out_.fields.size(),
        static_cast<unsigned long>(imu_count_));
      t_last_log_ = t;
      frames_at_log_ = frames_;
    }
  }

  // ---- 配置 ----
  std::string in_topic_, out_topic_, target_frame_;
  int64_t stamp_offset_ms_{0};
  double timestamp_scale_{1e9};      // 逐点 timestamp 换算系数：禾赛秒 → 纳秒
  bool qos_reliable_{false};
  double log_interval_{2.0};

  // ---- 首帧探测出来的字段偏移（-1 = 该字段不存在）----
  int ring_off_{-1};
  int tag_off_{-1};
  int ts_off_{-1};
  bool layout_checked_{false};

  // ---- IMU 转发 ----
  bool relay_imu_{true};
  std::string in_imu_topic_, out_imu_topic_;

  // ---- 运行期对象与统计 ----
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  PointCloud2 out_;                  // 复用的输出消息
  Imu imu_out_;                      // 复用的 IMU 消息
  uint64_t imu_count_{0};
  uint64_t frames_{0}, frames_at_log_{0}, pts_{0};
  rclcpp::Time t_last_log_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jt128LivoxCompat>());
  rclcpp::shutdown();
  return 0;
}
