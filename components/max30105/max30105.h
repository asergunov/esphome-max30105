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
  void recoverConfiguration();

protected:
  template <typename REG> bool read(REG &reg) {
    const auto prevValue = static_cast<uint8_t>(reg);
    auto &&result =
        i2c::I2CDevice::read_byte(REG::REG_ADR, &static_cast<uint8_t &>(reg));
    if (result) {
      ESP_LOGVV(TAG, "Read %s(%02X): %02X", RegTraits<REG>::name,
                RegTraits<REG>::address, static_cast<uint8_t>(reg));
      if (prevValue != static_cast<uint8_t>(reg)) {
        ESP_LOGV(TAG, "Value changed %s(%02X): %02X -> %02X",
                 RegTraits<REG>::name, RegTraits<REG>::address, prevValue,
                 static_cast<uint8_t>(reg));
      }
    } else {
      ESP_LOGE(TAG, "Can't read %s(%02X) register.", RegTraits<REG>::name,
               RegTraits<REG>::address);
      status_set_error();
    }
    return result;
  }
  template <typename REG> bool write(const REG &reg) {
    auto &&result = i2c::I2CDevice::write_byte(REG::REG_ADR, reg);
    if (result) {
      ESP_LOGV(TAG, "Wrote %s(%02X): %02X", RegTraits<REG>::name,
               RegTraits<REG>::address, static_cast<uint8_t>(reg));
    } else {
      ESP_LOGE(TAG, "Can't write %s(%02X) register.", RegTraits<REG>::name,
               RegTraits<REG>::address);
      status_set_error();
    }
    return result;
  }

  // Config
  uint16_t _pointLimit = 8;

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


  FIFO_RD_PTR::REG rdReg;
  FIFO_WR_PTR::REG wrReg;

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