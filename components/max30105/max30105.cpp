#include "max30105.h"

#include "esphome/components/max30105/max30105_registers.h"
#include "max30105_registers.h"
#include <cassert>
#include <cstdint>

namespace esphome {

namespace max30105 {

MAX30105Sensor::MAX30105Sensor() {
  _config << FIFO_ROLLOVER_EN(true) << SMP_AVE(32) << MODE(Mode::MultiLed)
          << ADC_RGE::fromFulScale(4096) << SR(50)
          << LED_PW::fromPulseWidth(uint16_t(411)) << LED1_PA(0x1f)
          << LED2_PA(0x1f) << LED3_PA(0x1f) << PILOT_PA(0x1f)
          << SLOT1(Slot::LedRed) << SLOT2(Slot::LedGreen) << SLOT3(Slot::LedIR)
          << SLOT4(Slot::Disabled) << FIFO_RD_PTR(0) << FIFO_WR_PTR(0)
          << OVF_COUNTER(0);
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
  softReset([config = _config, this]() mutable {
    config << FIFO_RD_PTR(0) << FIFO_WR_PTR(0) << OVF_COUNTER(0);
    for_each_in_tuple(config, [&](auto &reg) {
      using REG = std::decay_t<decltype(reg)>;
      using traits = RegTraits<REG>;
      const auto &value = config.reg<REG>();
      if (!write(value))
        return;
      if constexpr (std::is_same_v<REG, FIFO_RD_PTR::REG> ||
                    std::is_same_v<REG, FIFO_WR_PTR::REG> ||
                    std::is_same_v<REG, OVF_COUNTER::REG>)
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
  });
}

void MAX30105Sensor::update() {
  auto publish_state = [](SensorData &sensor, Data &data) {
    if (sensor.sensor && sensor.sent_counter != data.counter &&
        !data.buffer.empty()) {
      ESP_LOGD(TAG, "Publishing sensor state");
      sensor.sensor->publish_state(data.buffer.back());
      sensor.sent_counter = data.counter;
    } else {
      ESP_LOGV(TAG, "Sensor not published");
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
    if (!this->read(_config.reg<MODE::REG>()))
      return;
    if (_config.field<MODE>() == Mode::MultiLed) {
      if (!this->read(_config.reg<MultiLedMode1>()))
        return;
      if (!this->read(_config.reg<MultiLedMode2>()))
        return;
    }

    auto &rdReg = _config.reg<FIFO_RD_PTR::REG>();
    auto &wrReg = _config.reg<FIFO_WR_PTR::REG>();
    if (!this->read(rdReg)) {
      return;
    }
    if (!this->read(wrReg)) {
      return;
    }

    const auto &samplesToRead =
        static_cast<uint8_t>(FIFO_WR_PTR(wrReg) - FIFO_RD_PTR(rdReg)) % 32;
    if (samplesToRead == 0) {
      // nothing to read
      return;
    }

    const auto &ledSlots = _config.ledSlots();
    const auto &numLeds = [&ledSlots] {
      uint8_t result = 0;
      for (const auto &slot : ledSlots) {
        switch (slot) {
        case Slot::Led1:
        case Slot::Led2:
        case Slot::Led3:
        case Slot::Led1Pilot:
        case Slot::Led2Pilot:
        case Slot::Led3Pilot:
          ++result;
          break;
        }
      }
      return result;
    }();
    const uint16_t bytesToRead = samplesToRead * numLeds * 3;

    ESP_LOGV(TAG,
             "Samples to read: %u. Bytes to read: %u. NumLeds: %u. Led Slots: "
             "(%u, %u, %u, %u).",
             samplesToRead, bytesToRead, numLeds, ledSlots[0], ledSlots[1],
             ledSlots[2], ledSlots[3]);

    uint8_t buffer[bytesToRead];
    if (bytesToRead > sizeof(buffer)) {
      ESP_LOGE(TAG,
               "Buffer too small for available data. Want to read %u bytes, "
               "but buffer size is %u. This means there is an error in finding "
               "out buffer size.",
               bytesToRead, sizeof(buffer));
      status_set_error();
      return;
    }
    if (!this->read_bytes(FIFO_DATA::REG_ADR, buffer, bytesToRead)) {
      ESP_LOGE(TAG, "Can't read FIFO DATA");
      status_set_error();
      return;
    }

    auto *p = buffer;
    auto decode_to = [&](Data &data) {
      auto &&container = data.buffer;
      uint32_t result = 0;
      for (uint8_t i = 0; i < 3; ++i)
        result = (result << 8) + *(p++);
      container.push_back(result);
      ++data.counter;
      while (container.size() > _pointLimit)
        container.pop_front();
      ESP_LOGV(
          TAG, "Decode %s value: %u. Counter %u. Queue size: %u",
          [&] {
            if (&data == &red_)
              return "Red";
            else if (&data == &green_)
              return "Green";
            else if (&data == &ir_)
              return "IR";
            return "Unknown";
          }(),
          result, data.counter, container.size());
    };

    for (const auto &slot : ledSlots) {
      switch (slot) {
      case Slot::LedRed:
      case Slot::LedRedPilot:
        decode_to(red_);
        break;
      case Slot::LedGreen:
      case Slot::LedGreenPilot:
        decode_to(green_);
        break;
      case Slot::LedIR:
      case Slot::LedIRPilot:
        decode_to(ir_);
        break;
      }
    }
  }
  status_clear_error();
}

} // namespace max30105
} // namespace esphome