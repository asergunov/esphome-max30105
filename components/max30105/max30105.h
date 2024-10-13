#pragma once

#include "esp_log.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "max30105_registers.h"
#include <cstdint>
#include <deque>

namespace esphome {
namespace max30105 {

static const char *const TAG = "max44009.sensor";
using namespace max30105_registers;

class MAX30105Sensor : public PollingComponent, public i2c::I2CDevice {

public:
  MAX30105Sensor();

  void setup() override;
  void update() override;
  void loop() override;

  bool softReset(std::function<void()> doAfterReset);

  void set_red_sensor(sensor::Sensor *red_sensor) {
    red_sensor_.sensor = red_sensor;
  }
  void set_green_sensor(sensor::Sensor *green_sensor) {
    green_sensor_.sensor = green_sensor;
  }
  void set_ir_sensor(sensor::Sensor *ir_sensor) {
    ir_sensor_.sensor = ir_sensor;
  }

  void dump_config() override;

protected:
  void recoverConfiguration();

  template <typename T> bool read(T &reg) {
    return i2c::I2CDevice::read_byte(T::REG_ADR, &static_cast<uint8_t &>(reg));
  }
  template <typename T> bool write(const T &reg) {
    return i2c::I2CDevice::write_byte(T::REG_ADR, reg);
  }

  // Config
  uint16_t _pointLimit = 32;

  // State
  enum State { Ready, Reseting, Sampling };
  State _state = State::Ready;
  bool _needReset = false;
  std::function<void()> _doAfterReset;

  // Registers value we like to have saved
  PART_ID _partId;
  REV_ID _revisionId;
  Configuration _config;

  using counter_type = uint32_t;
  // Buffers
  struct Data {
    std::deque<uint32_t> buffer;
    counter_type counter = 0;
  };
  Data red_;
  Data green_;
  Data ir_;

  struct SensorData {
    sensor::Sensor *sensor{nullptr};
    counter_type sent_counter = 0;
  };

  SensorData red_sensor_;
  SensorData green_sensor_;
  SensorData ir_sensor_;
};

} // namespace max30105
} // namespace esphome