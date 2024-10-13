#include "max30105.h"

#include "max30105_registers.h"
#include <cassert>
#include <cstdint>

namespace esphome {

namespace max30105 {

MAX30105Sensor::MAX30105Sensor() {
  _config << FIFO_ROLLOVER_EN(true) << SMP_AVE(4) << MODE(Mode::MultiLed)
          << ADC_RGE::fromFulScale(4096) << SR(400)
          << LED_PW::fromPulseWidth(uint16_t(411)) << LED1_PA(0x1f)
          << LED2_PA(0x1f) << LED3_PA(0x1f) << PILOT_PA(0x1f)
          << SLOT1(Slot::LedRed) << SLOT2(Slot::LedGreen) << SLOT3(Slot::LedIR)
          << SLOT4(Slot::Disabled);
}

void MAX30105Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX30105:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX30105 failed!");
  }
  ESP_LOGCONFIG(TAG, "  PartID: %02X", static_cast<uint8_t>(_partId));
  ESP_LOGCONFIG(TAG, "  RevisionId: %02X", static_cast<uint8_t>(_revisionId));
  for_each_in_tuple(_config, [](const auto &reg) {
    using traits = RegTraits<std::decay_t<decltype(reg)>>;
    ESP_LOGCONFIG(TAG, "  %s(%02X): %02X", traits::name, traits::address,
                  static_cast<uint8_t>(reg));
  });
  ESP_LOGCONFIG(TAG, "  State: %s", [&] {
    switch (_state) {
    case Ready:
      return "Ready";
    case Reseting:
      return "Reseting";
    case Sampling:
      return "Sampling";
    }
    return "Unknown";
  }());
  ESP_LOGCONFIG(TAG, "  Reset Requested: %01X", _needReset);

  LOG_SENSOR("  ", "Red", this->red_sensor_.sensor);
  LOG_SENSOR("  ", "Green", this->green_sensor_.sensor);
  LOG_SENSOR("  ", "Infrared", this->ir_sensor_.sensor);
}

void MAX30105Sensor::setup() {
  if (!this->read(_partId)) {
    ESP_LOGE(TAG, "Can't read PART_ID");
    status_set_error();
    return;
  }
  ESP_LOGD(TAG, "Read ParID: %02X", static_cast<uint8_t>(_partId));
  if (_partId != PART_ID::POR_STATE) {
    ESP_LOGE(TAG, "PART_ID is not the one expected");
    status_set_error();
    return;
  }

  if (!this->read(_revisionId)) {
    ESP_LOGE(TAG, "Can't red REV_ID");
    status_set_error();
    return;
  }
  ESP_LOGD(TAG, "Read RevisionID: %02X", static_cast<uint8_t>(_revisionId));

  recoverConfiguration();
}

void MAX30105Sensor::recoverConfiguration() {
  _needReset = false;
  _state = Ready;
  softReset([config = _config, this] {
    for_each_in_tuple(config, [&](auto &reg) {
      using REG = std::decay_t<decltype(reg)>;
      using traits = RegTraits<REG>;
      const auto &value = config.reg<REG>();
      if (!write(value))
        return;
      auto &storage = _config.reg<REG>();
      if (!read(storage))
        return;
      if (storage != value) {
        ESP_LOGW(TAG,
                 "Read %s back is different. Expected: %02X, "
                 "Actual: %02X",
                 traits::name, value, storage);
      }
    });

    ESP_LOGD(TAG, "Resetting Read Pointer");
    FIFO_RD_PTR::REG rdReg;
    rdReg << FIFO_RD_PTR(0);
    if (!this->write(rdReg))
      return;

    ESP_LOGD(TAG, "Resetting Write Pointer");
    FIFO_WR_PTR::REG wrReg;
    wrReg << FIFO_WR_PTR(0);
    if (!this->write(wrReg))
      return;

    ESP_LOGD(TAG, "Resetting Overflow Counter");
    OVF_COUNTER::REG ovReg;
    ovReg << OVF_COUNTER(0);
    if (!this->write(ovReg))
      return;
  });
}

void MAX30105Sensor::update() {
  auto publish_state = [](SensorData &sensor, Data &data) {
    if (sensor.sensor && sensor.sent_counter != data.counter) {
      sensor.sensor->publish_state(data.buffer.back());
    }
  };
  publish_state(red_sensor_, red_);
  publish_state(green_sensor_, green_);
  publish_state(ir_sensor_, ir_);
}

bool MAX30105Sensor::softReset(std::function<void()> doAfterReset) {
  if (_needReset) {
    ESP_LOGW(TAG, "Reset already scheduled");
    return false;
  }
  if (_state == State::Reseting) {
    ESP_LOGW(TAG, "Reset in progress");
    return false;
  }

  _needReset = true;
  _doAfterReset = doAfterReset;
  return true;
}

void MAX30105Sensor::loop() {
  if (_state == Reseting) {
    if (!read(_config.reg<RESET::REG>())) {
      return;
    }
    if (_config.field<RESET>()) {
      ESP_LOGD(TAG, "Waiting for reset complete");
      return;
    }
    ESP_LOGD(TAG, "Reset is done");
    if (_doAfterReset) {
      ESP_LOGD(TAG, "Performing after reset actions");
      _doAfterReset();
      _doAfterReset = {};
    }
    _state = Ready;
  }

  InterruptStatus1 int1;
  if (!read(int1)) {
    return;
  }

  if (PWR_RDY(int1)) {
    ESP_LOGD(TAG, "Power Ready");
    ESP_LOGW(TAG, "Looks like we had undervoltage. Recovering configuration.");
    recoverConfiguration();
    return;
  }

  const A_FULL almostFull(int1);
  const DATA_RDY dataReady(int1);
  const ALC_OVF ambientLightCancellationOverflow(int1);
  const PROX_INT proximityThreshold(int1);
  if (almostFull) {
    ESP_LOGD(TAG, "Almost Full");
  }
  if (dataReady) {
    ESP_LOGD(TAG, "Almost Full");
  }
  if (ambientLightCancellationOverflow) {
    ESP_LOGD(TAG, "Ambient Light Cancellation Overflow");
  }
  if (proximityThreshold) {
    ESP_LOGD(TAG, "Proximity Threshold");
  }

  if (_needReset) {
    auto &reg = _config.reg<RESET::REG>();
    reg << RESET(true);
    if (!write(reg)) {
      return;
    }
    _state = Reseting;
    _needReset = false;
    return;
  }

  InterruptStatus2 int2;
  if (!read(int2)) {
    return;
  }

  const DIE_TEMP_RDY dieTemperatureReady(int2);
  if (dieTemperatureReady) {
    ESP_LOGD(TAG, "Die Temperature Ready");
  }

  if (true || dataReady) {
    FIFO_RD_PTR::REG rdReg;
    FIFO_WR_PTR::REG wrReg;
    if (!this->read(rdReg)) {
      return;
    }
    if (!this->read(wrReg)) {
      return;
    }

    const uint8_t samplesToRead =
        (FIFO_WR_PTR(wrReg) - FIFO_RD_PTR(rdReg)) % 32;
    if (samplesToRead == 0) {
      // nothing to read
      return;
    }

    const auto numLeds = _config.field<MODE>().numLeds();
    const uint16_t bytesToRead = samplesToRead * numLeds * 3;

    ESP_LOGV(TAG, "Samples to read: %ui. Bytes to read: %ui", samplesToRead, bytesToRead);
    
    uint8_t buffer[32 * 3 * 12];
    //   assert(bytesToRead < sizeof(buffer));
    if (!this->read_bytes(FIFO_DATA::REG_ADR, buffer, bytesToRead)) {
      ESP_LOGE(TAG, "Can't read FIFO DATA");
      status_set_error();
      return;
    }

    auto *p = buffer;
    auto decode_to = [&](Data &data) {
      auto container = data.buffer;
      uint32_t result = 0;
      for (uint8_t i = 0; i < 3; ++i)
        result = (result << 8) + *(p++);
      container.push_back(result);
      ++data.counter;
      ESP_LOGV(TAG, "Decode value: %ui. Counter %ui", result, data.counter);
      while (container.size() > _pointLimit)
        container.pop_front();
    };

    for (uint8_t i = 0; i < samplesToRead; ++i) {
      decode_to(red_);
      if (numLeds > 1)
        decode_to(green_);
      if (numLeds > 2)
        decode_to(ir_);
    }
  }
}

} // namespace max30105
} // namespace esphome