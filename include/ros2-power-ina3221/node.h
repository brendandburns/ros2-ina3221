#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;

class INA3221Publisher : public rclcpp::Node
{
public:
  INA3221Publisher();
  ~INA3221Publisher();

  void init_param();

private:
  void _init();
  void _loop();
  void _reset();

  void enable_channel(uint8_t channel);

  int _i2c_read_word_msb(uint32_t addr);
  int _i2c_write_word_msb(uint32_t addr, uint16_t val);

  bool _initialized;
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr _publisher;

  int _i2c_handle;

  std::string _i2c_device;
  std::string _topic;
  int _channel;
  float _charge_voltage;

  std::string _serial_number;
  std::string _location;
  float _design_capacity;
};
