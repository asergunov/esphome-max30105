#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
// #include "esphome/core/ring_buffer.h"

#include "max30105_registers.h"
#include <cstdint>
#include <deque>

namespace esphome {
namespace max30105 {

using namespace max30105_registers;
class MAX30105Sensor : public sensor::Sensor,
                       public PollingComponent,
                       public i2c::I2CDevice {

public:
  MAX30105Sensor();

  void setup() override;
  void update() override;
  void loop() override;

  bool softReset();

  void set_red_sensor(sensor::Sensor *red_sensor) { red_sensor_ = red_sensor; }
  void set_green_sensor(sensor::Sensor *green_sensor) { green_sensor_ = green_sensor; }
  void set_ir_sensor(sensor::Sensor *ir_sensor) { ir_sensor_ = ir_sensor; }

protected:
  template <typename T> bool read(T &reg) {
    return i2c::I2CDevice::read_byte(T::REG_ADR, &static_cast<uint8_t&>(reg));
  }
  template <typename T> bool write(const T &reg) {
    return i2c::I2CDevice::write_byte(T::REG_ADR, reg);
  }

  // Config
  uint16_t _pointLimit = 32;

  // State
  bool _resetInProgress = false;
  std::function<void()> _doAfterReset;

  // Registers value we like to have saved
  PART_ID _partId;
  REV_ID _revisionId;
  InterruptEnable1 _interruptEnab1e1;
  InterruptEnable2 _interruptEnab1e2;
  ModeConfiguration _modeConfiguration;
  FIFOConfiguration _fifoConfiguration;
  SpO2Configuration _sp02Configuration;

  // Buffers
  std::deque<uint32_t> _red;
  std::deque<uint32_t> _green;
  std::deque<uint32_t> _ir;

  sensor::Sensor *red_sensor_{nullptr};
  sensor::Sensor *green_sensor_{nullptr};
  sensor::Sensor *ir_sensor_{nullptr};
};

} // namespace max30105
} // namespace esphome