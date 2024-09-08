#include <fstream>
#include <sstream>

#include <tobas_std_tools/string.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/cpu.hpp>

using namespace std;

class CpuHandlerNode : public tobas::BaseNode
{
  static constexpr auto kSamplingPeriod = 1s;
  static constexpr char kTemperatureFilePath[] = "/sys/class/thermal/thermal_zone0/temp";
  static constexpr char kStatisticsFilePath[] = "/proc/stat";

  using self = CpuHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit CpuHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  int temp_millidegrees_;
  std::string cpu_line_, token_;
  uint64_t prev_user_time_ = 0, prev_nice_time_ = 0, prev_system_time_ = 0, prev_idle_time_ = 0;

  // Publisher
  ros2::PublisherPtr<tobas_msgs::msg::Cpu> cpu_pub_;

  // Timer
  ros2::TimerPtr main_timer_;

  bool getTemperature(double& temp);
  bool getFrequency(uint64_t& freq);
  bool getLoad(double& load);

  void mainTimerCb();
};

CpuHandlerNode::CpuHandlerNode(const rclcpp::NodeOptions& options) : super("cpu_handler", options)
{
  cpu_pub_ = createPublisher<tobas_msgs::msg::Cpu>(tobas::kCpuTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

bool CpuHandlerNode::getTemperature(double& temp)
{
  ifstream temp_file(kTemperatureFilePath);
  if (!temp_file)
  {
    TOBAS_ERROR("Failed to open ", kTemperatureFilePath, ".");
    return false;
  }
  temp_file >> temp_millidegrees_;
  temp = static_cast<double>(temp_millidegrees_) * 1e-3;
  return true;
}

bool CpuHandlerNode::getFrequency(uint64_t& freq)
{
  const auto vcgencmd_out = linux::executeCommand("vcgencmd measure_clock arm");
  const auto freq_str = tobas_std::split(vcgencmd_out, '=').back();  // 数値部分のみ抜き出す
  freq = stoul(freq_str);                                            // str -> uint64
  return true;
}

bool CpuHandlerNode::getLoad(double& load)
{
  // ファイルを読み込む
  ifstream stat_file(kStatisticsFilePath);
  if (!stat_file)
  {
    TOBAS_ERROR("Failed to open ", kStatisticsFilePath, ".");
    return false;
  }

  // ファイルの最初の行を読む
  if (!getline(stat_file, cpu_line_))
  {
    TOBAS_ERROR("Failed to read the first line of ", kStatisticsFilePath, ".");
    return false;
  }

  // CPUの行を空白で区切る
  istringstream iss(cpu_line_);

  // CPU使用時間を取得 (http://my-web-site.iobb.net/~yuki/2017-10/raspberry-pi/cpustat/)
  // 最初のトークン（"cpu"）を読み飛ばす
  iss >> token_;

  // (01) Time spent in user mode
  if (!(iss >> token_))
  {
    TOBAS_ERROR("Failed to read the CPU time spent in user mode.");
    return false;
  }
  const auto new_user_time = stoul(token_);

  // (02) Time spent in user mode with low priority (nice)
  if (!(iss >> token_))
  {
    TOBAS_ERROR("Failed to read the CPU time spent in user mode with low priority.");
    return false;
  }
  const auto new_nice_time = stoul(token_);

  // (03) Time spent in system mode
  if (!(iss >> token_))
  {
    TOBAS_ERROR("Failed to read the CPU time spent in system mode.");
    return false;
  }
  const auto new_system_time = stoul(token_);

  // (04) Time spent in the idle task
  if (!(iss >> token_))
  {
    TOBAS_ERROR("Failed to read the CPU time spent in the idle task.");
    return false;
  }
  const auto new_idle_time = stoul(token_);

  // 前回値との差分を計算
  const auto user_time = new_user_time - prev_user_time_;
  const auto nice_time = new_nice_time - prev_nice_time_;
  const auto system_time = new_system_time - prev_system_time_;
  const auto idle_time = new_idle_time - prev_idle_time_;

  // 負荷を計算
  const auto busy_time = user_time + nice_time + system_time;
  const auto all_time = busy_time + idle_time;
  load = static_cast<double>(busy_time) / static_cast<double>(all_time);

  // CPU使用時間を更新
  prev_user_time_ = new_user_time;
  prev_nice_time_ = new_nice_time;
  prev_system_time_ = new_system_time;
  prev_idle_time_ = new_idle_time;

  return true;
}

void CpuHandlerNode::mainTimerCb()
{
  // Create ROS message
  auto cpu_msg = std::make_unique<tobas_msgs::msg::Cpu>();
  cpu_msg->header.stamp = get_clock()->now();

  // Get CPU temperature
  if (!getTemperature(cpu_msg->temperature))
    return;

  // Get CPU frequency
  if (!getFrequency(cpu_msg->frequency))
    return;

  // Get CPU load
  if (!getLoad(cpu_msg->load))
    return;

  // Publish ROS message
  cpu_pub_->publish(move(cpu_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(CpuHandlerNode)
