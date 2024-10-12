#include "max30105.h"

#include "esp_log.h"

#include "esphome/core/log.h"
#include "max30105_registers.h"
#include <cassert>
#include <cstdint>

namespace esphome {

namespace max30105 {


MAX30105Sensor::MAX30105Sensor() {
  FIFO_ROLLOVER_EN rolloveEnabled(_fifoConfiguration);
  rolloveEnabled = true;

  MODE fifoMode(_modeConfiguration);
  fifoMode = MODE::MultiLed;

  ADC_RGE adcRange(_sp02Configuration);
  adcRange = 4096;

  SR sampleRate(_sp02Configuration);
  sampleRate = 400;

  LED_PW ledPulseWidth(_sp02Configuration);
  ledPulseWidth = 411;
}

void MAX30105Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX30105:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX30105 failed!");
  }
  ESP_LOGCONFIG(TAG, "PartID: %02X", static_cast<uint8_t>(_partId));
  ESP_LOGCONFIG(TAG, "RevisionId: %02X", static_cast<uint8_t>(_revisionId));
  ESP_LOGCONFIG(TAG, "InterruptEnable1: %02X",
                static_cast<uint8_t>(_interruptEnab1e1));
  ESP_LOGCONFIG(TAG, "InterruptEnable2: %02X",
                static_cast<uint8_t>(_interruptEnab1e2));
  ESP_LOGCONFIG(TAG, "ModeConfiguration: %02X",
                static_cast<uint8_t>(_modeConfiguration));
  ESP_LOGCONFIG(TAG, "FIFOConfiguration: %02X",
                static_cast<uint8_t>(_fifoConfiguration));
  ESP_LOGCONFIG(TAG, "SP02Configuration: %02X",
                static_cast<uint8_t>(_sp02Configuration));
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
  _state = _state == Off ? Off : Ready;
  softReset([fifoConfiguration = _fifoConfiguration,
             modeConfiguration = _modeConfiguration,
             sp02Configuration = _sp02Configuration, this] {
    writeConfig(_fifoConfiguration, fifoConfiguration, "Fifo Configuration");
    writeConfig(_modeConfiguration, modeConfiguration, "Mode Configuration");
    writeConfig(_sp02Configuration, sp02Configuration, "Sp02 Configuration");

    ESP_LOGD(TAG, "Resetting Read Pointer");
    FIFO_RD_PTR::REG rdReg;
    auto rdPtr = FIFO_RD_PTR(rdReg);
    rdPtr = 0;
    if (!this->write(rdReg)) {
      ESP_LOGE(TAG, "Can't write FIFO Read Pointer");
      status_set_error();
    }

    ESP_LOGD(TAG, "Resetting Write Pointer");
    FIFO_WR_PTR::REG wrReg;
    auto wrPtr = FIFO_WR_PTR(wrReg);
    wrPtr = 0;
    if (!this->write(wrReg)) {
      ESP_LOGE(TAG, "Can't write FIFO Write Pointer");
      status_set_error();
    }

    ESP_LOGD(TAG, "Resetting Overflow Counter");
    OVF_COUNTER::REG ovReg;
    auto ovPtr = OVF_COUNTER(ovReg);
    ovPtr = 0;
    if (!this->write(ovReg)) {
      ESP_LOGE(TAG, "Can't write Overflow Counter");
      status_set_error();
    }
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

  _doAfterReset = doAfterReset;
  return true;
}

void MAX30105Sensor::loop() {
  if (_state == Reseting) {
    if (!read(_modeConfiguration)) {
      ESP_LOGE(TAG, "Can't get FIFO_RD_PTR");
      status_set_error();
      return;
    }
    if (RESET(_modeConfiguration)) {
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
    ESP_LOGE(TAG, "Can't get InterruptStatus1");
    status_set_error();
    return;
  }

  const PWR_RDY powerReady(int1);
  if (powerReady) {
    ESP_LOGD(TAG, "Power Ready");
    _state = Ready;
    if(_state != Off) {
      ESP_LOGW(TAG, "Looks like we had undervoltage. Recovering configuration.");
      recoverConfiguration();
      return;
    }
  }
  
  const A_FULL almostFull(int1);
  const DATA_RDY dataReady(int1);
  const ALC_OVF ambientLightCancellationOverflow(int1);
  const PROX_INT proximityThreshold(int1);
  if(almostFull) {
    ESP_LOGD(TAG, "Almost Full");
  }
  if(dataReady) {
    ESP_LOGD(TAG, "Almost Full");
  }
  if(ambientLightCancellationOverflow) {
    ESP_LOGD(TAG, "Ambient Light Cancellation Overflow");
  }
  if(proximityThreshold) {
    ESP_LOGD(TAG, "Proximity Threshold");
  }

  if (_needReset && _state != Off) {
    RESET reset(_modeConfiguration);
    reset = true;
    if (!write(_modeConfiguration)) {
      ESP_LOGE(TAG, "Cant write Mode Configuration");
      status_set_error();
      return;
    }
    _state = Reseting;
    _needReset = false;
    return;
  }

  InterruptStatus2 int2;
  if (!read(int2)) {
    ESP_LOGE(TAG, "Can't get InterruptStatus2");
    status_set_error();
    return;
  }

  const DIE_TEMP_RDY dieTemperatureReady(int2);
  if(dieTemperatureReady){
    ESP_LOGD(TAG, "Die Temperature Ready");
  }

  if(dataReady) {
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
    auto decode_to = [&](Data &data) {
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
}

} // namespace max30105
} // namespace esphome