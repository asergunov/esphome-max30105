#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace max30105 {

class MAX30105Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice
{
    
};

}
}