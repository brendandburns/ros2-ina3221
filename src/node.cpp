#include "ros2-power-ina3221/node.h"

extern "C" {
#include <byteswap.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <i2c/smbus.h>
}

// Most of these constants are taken from https://raw.githubusercontent.com/adafruit/Adafruit_INA3221/refs/heads/main/Adafruit_INA3221.h
// Manufacturer ID for Texas Instruments
#define INA3221_MANUFACTURER_ID 0x5449 
// Die ID for INA3221
#define INA3221_DIE_ID 0x3220 

#define INA3221_REG_CONFIGURATION 0x00
#define INA3221_REG_SHUNTVOLTAGE_CH1 0x01 
#define INA3221_REG_BUSVOLTAGE_CH1 0x02
#define INA3221_REG_SHUNTVOLTAGE_CH2 0x03
#define INA3221_REG_BUSVOLTAGE_CH2 0x04
#define INA3221_REG_SHUNTVOLTAGE_CH3 0x05
#define INA3221_REG_BUSVOLTAGE_CH3 0x06
#define INA3221_REG_MANUFACTURER_ID 0xFE
#define INA3221_REG_DIE_ID 0xFF 

INA3221Publisher::~INA3221Publisher()
{
  if (this->_initialized)
  {
    close(this->_i2c_handle);
  }
}

void INA3221Publisher::init_param()
{
  this->declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
  this->declare_parameter<int>("channel", 0);
  this->declare_parameter<std::string>("topic", "/power");
  this->declare_parameter<float>("charge_voltage", 13.0);
  this->declare_parameter<std::string>("location", "unknown");
  this->declare_parameter<std::string>("serial_number", "unknown");
  this->declare_parameter<float>("design_capacity_amps", 10.0);

  this->get_parameter_or<std::string>("i2c_device", _i2c_device, "/dev/i2c-1");
  this->get_parameter_or<std::string>("topic", _topic, "/power");
  this->get_parameter_or<int>("channel", _channel, 0);
  this->get_parameter_or<float>("charge_voltage", _charge_voltage, 13.0);
  this->get_parameter_or<std::string>("location", _location, "unknown");
  this->get_parameter_or<std::string>("serial_number", _serial_number, "unknown");
  this->get_parameter_or<float>("design_capacity_amps", _design_capacity, 10.0);
}

int INA3221Publisher::_i2c_read_word_msb(uint32_t addr)
{
   auto res = i2c_smbus_read_word_data(this->_i2c_handle, addr);
   if (res < 0) {
     RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to read i2c register: " << std::hex << addr);
     return res;
   }
   return bswap_16(res);
}

int INA3221Publisher::_i2c_write_word_msb(uint32_t addr, uint16_t val)
{
   auto flip = bswap_16(val);
   auto res = i2c_smbus_write_word_data(this->_i2c_handle, addr, flip);
   if (res < 0) {
     RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to write i2c register: " << std::hex << addr << " with value: " << std::hex << val);
   }
   return res;
}

INA3221Publisher::INA3221Publisher() : Node("ina3221_node"), _initialized(false)
{
  this->init_param();
  if (this->_channel < 0 || this->_channel > 2)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Channel requested is out of range (0-2) was " << this->_channel);
    return;
  }
  this->_publisher = this->create_publisher<sensor_msgs::msg::BatteryState>(this->_topic, 10);
  auto timer_callback =
    [this]() -> void {
	      if (!this->_initialized) {
	        this->_init();
	      }
	      this->_loop();
      };
  _timer = this->create_wall_timer(10s, timer_callback);
}

void INA3221Publisher::_init()
{
  const char* device_path = this->_i2c_device.c_str();
  RCLCPP_INFO(this->get_logger(), "Initializing");
  this->_i2c_handle = open(device_path, O_RDWR);
  if (this->_i2c_handle < 0)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open device: " << this->_i2c_device << " (" << strerror(errno) << ")");
    return;
  }
  if(ioctl(this->_i2c_handle, I2C_SLAVE, 0x40) < 0) {
    RCLCPP_ERROR(this->get_logger(), "i2c device open failed");
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Opened i2c bus: " << device_path);

  auto manufacturer_id = this->_i2c_read_word_msb(INA3221_REG_MANUFACTURER_ID);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Read manufacturer id: " << std::hex << manufacturer_id);

  auto die_id = this->_i2c_read_word_msb(INA3221_REG_DIE_ID);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Read die id: " << std::hex << die_id);

  if (manufacturer_id != INA3221_MANUFACTURER_ID || die_id != INA3221_DIE_ID) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown device: " << std::hex << manufacturer_id << ", " << std::hex << die_id);
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Found an INA3221, configuring.");

  this->_reset();

  /*
   * For now just keep defaults.
   *
  for (int i = 0; i < 3; i++)
  {
    this->enable_channel(i);
  }

  uint16_t val = 0;
  // Bits 9-11 are the averaging mode
  // 000 -> 1 sample
  // 001 -> 4 samples
  // 010 -> 16 samples
  // 011 -> 64 samples
  // https://www.ti.com/lit/ds/symlink/ina3221.pdf for more values
  // Set to 4 samples for now.
  val |= 0b0000001000000000;

  // You can also configure the conversion time and mode, but
  // the defaults are fine for us for now.

  this->_i2c_write_word_msb(INA3221_REG_CONFIGURATION, val);
  */

  this->_initialized = true;
}

void INA3221Publisher::_reset()
{
  // Reset is the 15th bit
  this->_i2c_write_word_msb(INA3221_REG_CONFIGURATION, 0x8000);
}

void INA3221Publisher::enable_channel(uint8_t channel)
{
  if (channel > 2)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid channel: " << channel);
    return;
  }
  uint16_t val = 0;
  // Channel is bits 14 (0), 13 (1), 12 (2)
  switch (channel)
  {
    case 0:
      val = 0x4000;
      break;
    case 1:
      val = 0x2000;
      break;
    case 2:
      val = 0x100;
      break;
  };
  this->_i2c_write_word_msb(INA3221_REG_CONFIGURATION, val);
}

static const int bus_addrs[] = {INA3221_REG_BUSVOLTAGE_CH1, INA3221_REG_BUSVOLTAGE_CH2, INA3221_REG_BUSVOLTAGE_CH3};
static const int shunt_addrs[] = {INA3221_REG_SHUNTVOLTAGE_CH1, INA3221_REG_SHUNTVOLTAGE_CH2, INA3221_REG_SHUNTVOLTAGE_CH3};

void INA3221Publisher::_loop()
{
  auto val = this->_i2c_read_word_msb(bus_addrs[this->_channel]); 
  // From https://github.com/adafruit/Adafruit_INA3221/blob/main/Adafruit_INA3221.cpp#L142C3-L143C30
  float bus_voltage = (val >> 3) * 8e-3;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Voltage: " << val << ", " << bus_voltage);

  val = this->_i2c_read_word_msb(shunt_addrs[this->_channel]);
  // From https://github.com/adafruit/Adafruit_INA3221/blob/main/Adafruit_INA3221.cpp#L106
  float shunt_voltage = (val >> 3) * 40e-6;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Shunt Voltage: << " << val << ", " << shunt_voltage);

  auto message = sensor_msgs::msg::BatteryState();
  message.header.stamp = this->get_clock()->now();
  message.voltage = bus_voltage;
  message.current = shunt_voltage / 0.05 /* Ohms */;
  if (this->_charge_voltage < 0)
  {
    message.power_supply_status = 0; // Unknown
  } else if (bus_voltage > this->_charge_voltage) {
    message.power_supply_status = 1; // Charging
  } else {
    // This is somewhat imperfect, since very low batteries and weak chargers may leave
    // voltage below the minimum.
    message.power_supply_status = 2; // Discharging
  }
  message.location = this->_location;
  message.serial_number = this->_serial_number;
  message.design_capacity = this->_design_capacity;

  // Always present?
  message.present = true;

  // These are things that are unmeasured
  message.temperature = NAN;
  message.charge = NAN;
  message.capacity = NAN;

  message.power_supply_health = 0; // Unknown
  message.power_supply_technology = 0; // Unknown, TODO: make this configurable

  this->_publisher->publish(message);
}
