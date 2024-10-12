#include "max30105.h"

#include "esp_log.h"

#include "esphome/core/log.h"
#include "max30105_registers.h"
#include <cassert>
#include <cstdint>

namespace esphome {

namespace max30105 {

static const char *const TAG = "max44009.sensor";

MAX30105Sensor::MAX30105Sensor() {
  FIFO_ROLLOVER_EN rolloveEnabled(_fifoConfiguration);
  MODE fifoMode(_modeConfiguration);
  ADC_RGE adcRange(_sp02Configuration);
  SR sampleRate(_sp02Configuration);
  LED_PW ledPulseWidth(_sp02Configuration);

  rolloveEnabled = true;
  fifoMode = MODE::MultiLed;
  adcRange = 4096;
  sampleRate = 400;
  ledPulseWidth = 411;
}

void MAX30105Sensor::setup() {
  if (!this->read(_partId)) {
    ESP_LOGE(TAG, "Can't read PART_ID");
    status_set_error();
    return;
  }
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

  softReset([this] {
    ESP_LOGD(TAG, "Soft reset is done. Configuring.");
    if (!this->write(_fifoConfiguration)) {
      ESP_LOGE(TAG, "Can't write Fifo Configuration");
      status_set_error();
    }
    if (!this->write(_modeConfiguration)) {
      ESP_LOGE(TAG, "Can't write Mode Configuration");
      status_set_error();
    }
    if (!this->write(_sp02Configuration)) {
      ESP_LOGE(TAG, "Can't write SPo2 Configuration");
      status_set_error();
    }

    FIFO_RD_PTR::REG rdReg;
    FIFO_RD_PTR(rdReg) rdPtr;
    rdPtr = 0;
    if (!this->write(rdReg)) {
      ESP_LOGE(TAG, "Can't write FIFO Read Pointer");
      status_set_error();
    }


    FIFO_WR_PTR::REG wrReg;
    FIFO_WR_PTR(wrReg) wrPtr;
    wrPtr = 0;
    if (!this->write(wrReg)) {
      ESP_LOGE(TAG, "Can't write FIFO Write Pointer");
      status_set_error();
    }
    
    OVF_COUNTER::REG ovReg;
    OVF_COUNTER(ovReg) ovPtr;
    ovPtr = 0;
    if (!this->write(ovReg)) {
      ESP_LOGE(TAG, "Can't write Overflow Counter");
      status_set_error();
    }
  };
});

void MAX30105Sensor::update() {
  auto publish_state = [](SensorData& sensor, Data& data) {
    if(sensor.sensor && sensor.sent_counter != data.counter) {
      sensor.sensor->publish_state(data.buffer.back());
    }
  };
  publish_state(red_sensor_, red_);
  publish_state(green_sensor_, green_);
  publish_state(ir_sensor_, ir_);
}

bool MAX30105Sensor::softReset(std::function<void()> doAfterReset) {
  if (_resetInProgress) {
    ESP_LOGW(TAG, "Reset in progress");
    return false;
  }
  RESET reset(_modeConfiguration);
  reset = true;
  if (!write(_modeConfiguration)) {
    ESP_LOGE(TAG, "Cant write Mode Configuration");
    status_set_error();
    return false;
  }
  _doAfterReset = doAfterReset;
  _resetInProgress = true;
  return true;
}

void MAX30105Sensor::loop() {
  if (_resetInProgress) {
    if (!read(_modeConfiguration)) {
      ESP_LOGE(TAG, "Can't get FIFO_RD_PTR");
      status_set_error();
      return;
    }
    if (RESET(_modeConfiguration)) {
      return;
    }
    _resetInProgress = false;
    if (_doAfterReset) {
      _doAfterReset();
      _doAfterReset = {};
    }
  }

  FIFO_RD_PTR::REG rdReg;
  FIFO_WR_PTR::REG wrReg;
  if (!this->read(rdReg)) {
    ESP_LOGE(TAG, "Can't get FIFO_RD_PTR");
    status_set_error();
    return;
  }
  if (!this->read(wrReg)) {
    ESP_LOGE(TAG, "Can't get FIFO_WR_PTR");
    status_set_error();
    return;
  }

  const uint8_t samplesToRead = (FIFO_WR_PTR(wrReg) - FIFO_RD_PTR(rdReg)) % 32;
  if (samplesToRead == 0) {
    // nothing to read
    return;
  }

  const auto numLeds = MODE(_modeConfiguration).numLeds();

  const uint16_t bytesToRead = samplesToRead * numLeds * 3;
  uint8_t buffer[32 * 3 * 12];
  //   assert(bytesToRead < sizeof(buffer));
  if (!this->read_bytes(FIFO_DATA::REG_ADR, buffer, bytesToRead)) {
    ESP_LOGE(TAG, "Can't get DATA");
    status_set_error();
    return;
  }

  auto *p = buffer;
  auto decode_to = [&](Data& data) {
    auto container = data.buffer;
    uint32_t result = 0;
    for (uint8_t i = 0; i < 3; ++i)
      result = (result << 8) + *(p++);
    container.push_back(result);
    ++data.counter;
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

} // namespace max30105
} // namespace esphome