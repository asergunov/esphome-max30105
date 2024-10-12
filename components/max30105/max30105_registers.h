/// Datasheet
/// https://cdn.sparkfun.com/assets/learn_tutorials/5/7/7/MAX30105_3.pdf
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdint.h>

namespace max30105_registers {
namespace details {
template <typename T, size_t N>
size_t nextGratherOrEqualIndex(const std::array<T, N> &list, const T &value) {
  auto i = std::upper_bound(std::begin(list), std::end(list), value);
  if (i != std::begin(list))
    --i;
  return std::distance(std::begin(list), i);
}
} // namespace details

enum class Access { R, W, RW };

template <uint8_t _REG_ADR, Access _ACCESS = Access::RW,
          uint8_t _POR_STATE = 0x00>
struct Register {
  static constexpr auto REG_ADR = _REG_ADR;
  static constexpr auto POR_STATE = _POR_STATE;

  Register() : value(POR_STATE) {}
  Register(uint8_t v) : value(v) {}

  uint8_t value = POR_STATE;
  operator uint8_t &() { return value; }
  operator const uint8_t &() const { return value; }
};

template <typename _REG, uint8_t _LAST_BIT, uint8_t _FIRST_BIT> struct Field {
  static constexpr auto BEGIN_BIT = _FIRST_BIT;
  static constexpr auto END_BIT = _LAST_BIT + 1;
  static constexpr auto MASK = ((1u << END_BIT) - 1u) << BEGIN_BIT;
  using REG = _REG;
  REG &reg;
  Field(REG &_reg) : reg(_reg) {}

  operator uint8_t() const {
    return (static_cast<uint8_t>(reg) & MASK) >> BEGIN_BIT;
  }

  Field &operator=(const uint8_t value) {
    auto &reg_value = static_cast<uint8_t &>(reg);
    reg_value &= ~MASK;
    reg_value |= MASK & (value << BEGIN_BIT);
    return *this;
  }
};

template <typename _REG, uint8_t _BIT> using Bit = Field<_REG, _BIT, _BIT>;

/**
 * Whenever an interrupt is triggered, the MAX30105 pulls the active-low
 * interrupt pin into its low state until the interrupt is cleared.
 *
 * The interrupts are cleared whenever the interrupt status register is read, or
 * when the register that triggered the interrupt is read. For example, if the
 * particle-sensing sensor triggers an interrupt due to finishing a conversion,
 * reading either the FIFO data register or the interrupt register clears the
 * interrupt pin (which returns to its normal HIGH state). This also clears all
 * the bits in the interrupt status register to zero.
 *
 */
using InterruptStatus1 = Register<0x00, Access::R>;

/**
 * @brief FIFO Almost Full Flag
 *
 * In particle-sensing mode, this interrupt triggers when the FIFO write
 * pointer has a certain number of free spaces remaining. The trigger number
 * can be set by the FIFO_A_FULL[3:0] register. The interrupt is cleared by
 * reading the Interrupt Status 1 register (0x00).
 *
 */

using A_FULL = Bit<InterruptStatus1, 7>;

/**
 * @brief  New FIFO Data Ready
 *
 * In particle-sensing mode, this interrupt triggers when there is a new
 * sample in the data FIFO. The interrupt is cleared by reading the Interrupt
 * Status 1 register (0x00), or by reading the FIFO_DATA register.
 *
 */
using DATA_RDY = Bit<InterruptStatus1, 6>;

/**
 * @brief Ambient Light Cancellation Overflow
 *
 * This interrupt triggers when the ambient light cancellation function of the
 * particle-sensing photodiode has reached its maximum limit, and therefore,
 * ambient light is affecting the output of the ADC. The interrupt is cleared
 * by reading the Interrupt Status 1 register (0x00).
 *
 */
using ALC_OVF = Bit<InterruptStatus1, 5>;

/**
 * @brief Proximity Threshold Triggered
 *
 * The proximity interrupt is triggered when the proximity threshold is
 * reached, and particle-sensing mode has begun. This lets the host processor
 * know to begin running the particle-sensing algorithm and collect data. The
 * interrupt is cleared by reading the Interrupt Status 1 register (0x00).
 *
 */
using PROX_INT = Bit<InterruptStatus1, 4>;

/**
 * @brief : Power Ready Flag
 *
 * On power-up or after a brownout condition, when the supply voltage VDD
 * transitions from below the undervoltage-lockout (UVLO) voltage to above the
 * UVLO voltage, a power-ready interrupt is triggered to signal that the
 * module is powered-up and ready to collect data.
 *
 */
using PWR_RDY = Bit<InterruptStatus1, 0>;

using InterruptStatus2 = Register<0x01, Access::R>;

/**
 * @brief  Internal Temperature Ready Flag
 *
 * When an internal die temperature conversion is finished, this interrupt is
 * triggered so the processor can read the temperature data registers. The
 * interrupt is cleared by reading either the Interrupt Status 2 register
 * (0x01) or the TFRAC register (0x20).
 *
 */
using DIE_TEMP_RDY = Bit<InterruptStatus2, 1>;

/**
 * Interrupt Enable (0x02–0x03)
 *
 * Each source of hardware interrupt, with the exception of power ready, can be
 * disabled in a software register within the MAX30105 IC. The power-ready
 * interrupt cannot be disabled because the digital state of the module is reset
 * upon a brownout condition (low power supply voltage), and the default
 * condition is that all the interrupts are disabled. Also, it is important for
 * the system to know that a brownout condition has occurred, and the data
 * within the module is reset as a result. The unused bits should always be set
 * to zero for normal operation
 */
using InterruptEnable1 = Register<0x02>;
using A_FULL_EN = Bit<InterruptEnable1, 7>;
using DATA_RDY_EN = Bit<InterruptEnable1, 6>;

using ALC_OVF_EN = Bit<InterruptEnable1, 5>;
using PROX_INT_EN = Bit<InterruptEnable1, 4>;
using PWR_RDY_EN = Bit<InterruptEnable1, 0>;

using InterruptEnable2 = Register<0x03>;
using DIE_TEMP_RDY_EN = Bit<InterruptEnable2, 1>;

/**
 * @brief FIFO Write Pointer
The FIFO Write Pointer points to the location where the MAX30105 writes the next
sample. This pointer advances for each sample pushed on to the FIFO. It can also
be changed through the I2C interface when MODE[2:0] is 010, 011, or 111.
*/
using FIFO_WR_PTR = Field<Register<0x04>, 4, 0>;

/**
 * @brief FIFO Overflow Counter
 *
 * When the FIFO is full, samples are not pushed on to the FIFO, samples are
 * lost. OVF_COUNTER counts the number of samples lost. It saturates at 0xF.
 * When a complete sample is “popped” (i.e., removal of old FIFO data and
 * shifting the samples down) from the FIFO (when the read pointer advances),
 * OVF_COUNTER is reset to zero.
 *
 */
using OVF_COUNTER = Field<Register<0x05>, 4, 0>;

/**
 * @brief FIFO Read Pointer
 *
 * The FIFO Read Pointer points to the location from where the processor gets
 * the next sample from the FIFO through the I2C interface. This advances each
 * time a sample is popped from the FIFO. The processor can also write to this
 * pointer after reading the samples to allow rereading samples from the FIFO if
 * there is a data communication error.
 *
 */
using FIFO_RD_PTR = Field<Register<0x06>, 4, 0>;

/**
 * @brief FIFO Data Register
 *
 * The circular FIFO depth is 32 and can hold up to 32 samples of data. The
 * sample size depends on the number of LED channels configured as active. As
 * each channel signal is stored as a 3-byte data signal, the FIFO width can be
 * 3 bytes, 6 bytes, 9 bytes, or 12 bytes in size.
 *
 * The FIFO_DATA register in the I2C register map points to the next sample to
 * be read from the FIFO. FIFO_RD_PTR points to this sample. Reading the
 * FIFO_DATA register does not automatically increment the I2C register address.
 * Burst reading this register reads the same address over and over. Each sample
 * is 3 bytes of data per channel (i.e., 3 bytes for RED, 3 bytes for IR, etc.).
 *
 * The FIFO registers (0x04–0x07) can all be written and read, but in practice
 * only the FIFO_RD_PTR register should be written to in operation. The others
 * are automatically incremented or filled with data by the MAX30105. When
 * starting a new particle-sensing conversion, it is recommended to first clear
 * the FIFO_WR_PTR, OVF_COUNTER, and FIFO_RD_PTR registers to all zeroes (0x00)
 * to ensure the FIFO is empty and in a known state. When reading the MAX30105
 * registers in one burst-read I2C transaction, the register address pointer
 * typically increments so that the next byte of data sent is from the next
 * register, etc. The exception to this is the FIFO data register, register
 * 0x07. When reading this register, the address pointer does not increment, but
 * the FIFO_RD_PTR does. So the next byte of data sent represents the next byte
 * of data available in the FIFO.
 *
 * Entering and exiting the proximity mode (when PROX_INT_EN = 1) clears the
 * FIFO by setting the write and read pointers equal to each other
 *
 */

using FIFO_DATA = Register<0x07>;

using FIFOConfiguration = Register<0x08>;
/**
 * @brief Sample Averaging (SMP_AVE)
 *
 * To reduce the amount of data throughput, adjacent samples (in each individual
 * channel) can be averaged and decimated  on the chip by setting this register.
 *
 */
struct SMP_AVE : public Field<FIFOConfiguration, 7, 5> {
  SMP_AVE(FIFOConfiguration &reg) : Field(reg) {}
  SMP_AVE &operator=(uint8_t avarege) {
    const auto value = avarege >= 32   ? 0b101
                       : avarege >= 16 ? 0b100
                       : avarege >= 8  ? 0b011
                       : avarege >= 4  ? 0b010
                       : avarege > 2   ? 0x001
                                       : 0x00;
    Field::operator=(value);
    return *this;
  }
  operator uint8_t() const {
    auto value = Field::operator uint8_t();
    value = value > 0b0101 ? 0b101 : value;
    return 1 << value;
  }
};

/**
 * @brief  FIFO Rolls on Full (FIFO_ROLLOVER_EN)
 *
 *This bit controls the behavior of the FIFO when the FIFO becomes completely
 *filled with data. If FIFO_ROLLOVER_EN is set (1), the FIFO Address rolls over
 *to zero and the FIFO continues to fill with new data. If the bit is not set
 *(0), then the FIFO is not updated until FIFO_DATA is read or the WRITE/READ
 *pointer positions are changed.
 *
 */
using FIFO_ROLLOVER_EN = Bit<FIFOConfiguration, 4>;

/**
 * @brief FIFO Almost Full Value (FIFO_A_FULL)
 *
 * This register sets the trigger for the FIFO_A_FULL interrupt. For example, if
 * set to 0x0F, the interrupt triggers when there are 15 empty space left (17
 * data samples), and so on.
 *
 */
struct FIFO_A_FULL : Field<FIFOConfiguration, 3, 0> {
  static constexpr uint8_t decode(uint8_t value) { return 32 - value; }
};

using ModeConfiguration = Register<0x09>;
/**
 * @brief  Shutdown Control (SHDN)
 *
 * The part can be put into a power-save mode by setting this bit to one. While
 * in power-save mode, all registers retain their values, and write/read
 * operations function as normal. All interrupts are cleared to zero in this
 * mode.
 *
 */
using SHDN = Bit<ModeConfiguration, 7>;

/**
 * @brief Reset Control (RESET)
 *
 * When the RESET bit is set to one, all configuration, threshold, and data
 * registers are reset to their power-on-state through a power-on reset. The
 * RESET bit is cleared automatically back to zero after the reset sequence is
 * completed.
 *
 * Note: Setting the RESET bit does not trigger a PWR_RDY interrupt event.
 *
 */
using RESET = Bit<ModeConfiguration, 6>;

/**
 * @brief Mode Control
 *
 * These bits set the operating state of the MAX30105. Changing modes does not
 * change any other setting, nor does it erase any previously stored data inside
 * the data registers.
 *
 */
struct MODE : Field<ModeConfiguration, 2, 0> {
  enum Mode {
    None = 0x000,
    ParticleSensing1LED = 0b010,
    ParticleSensing2LED = 0b011,
    MultiLed = 0b111,
  };

  MODE(Field::REG &reg) : Field(reg) {}

  uint8_t numLeds() const {
    switch (static_cast<uint8_t>(*this)) {
    case ParticleSensing1LED:
      return 1;
    case ParticleSensing2LED:
      return 2;
    case MultiLed:
      return 3;
    default:
      return 0;
    }
  }

  MODE &operator=(Mode mode) {
    Field::operator=(mode);
    return *this;
  }

  operator Mode() const {
    switch (Field::operator uint8_t()) {
    case ParticleSensing1LED:
      return ParticleSensing1LED;
    case ParticleSensing2LED:
      return ParticleSensing2LED;
    case MultiLed:
      return MultiLed;
    default:
      return None;
    }
  }
};

using SpO2Configuration = Register<0x0A>;

/**
 * @brief : Particle-Sensing ADC Range Control
 *
 * This register sets the particle-sensing sensor ADC’s full-scale range as
 * shown in Table 5
 *
 */
struct ADC_RGE : Field<SpO2Configuration, 6, 5> {
  /**
   * @brief LSB SIZE (pA)
   *
   */
  static constexpr float LsbSize[] = {7.81, 15.63, 31.25, 62.5};

  /**
   * @brief FULL SCALE (nA)
   *
   */
  static constexpr std::array<uint16_t, 4> FullScale{2048, 4096, 8192, 16384};

  ADC_RGE(Field::REG &reg) : Field(reg) {}

  ADC_RGE operator=(uint16_t range) {
    const auto FullScale = ADC_RGE::FullScale; // constexpr std::upper_bound
                                               // available only in c++26
    Field::operator=(details::nextGratherOrEqualIndex(FullScale, range));
    return *this;
  }
};

/**
 * @brief : Particle-Sensing Sample Rate Control (Using 2 LEDs)
 *
 * These bits define the effective sampling rate with one sample consisting of
 * one IR pulse/conversion and one RED pulse/conversion. The sample rate and
 * pulse width are related in that the sample rate sets an upper bound on the
 * pulse width time. If the user selects a sample rate that is too high for the
 * selected LED_PW setting, the highest possible sample rate is programmed
 * instead into the register.
 *
 */
struct SR : Field<SpO2Configuration, 4, 2> {
  static constexpr std::array<uint16_t, 8> SamplesPerSecond{
      50, 100, 200, 400, 800, 1000, 1600, 3200};
  SR(Field::REG &reg) : Field(reg) {}
  SR &operator=(uint16_t sampleRate) {
    const auto SamplesPerSecond = SR::SamplesPerSecond;
    Field::operator=(
        details::nextGratherOrEqualIndex(SamplesPerSecond, sampleRate));
    return *this;
  }
};

/**
 * @brief  LED Pulse Width Control and ADC Resolution
 *
 * These bits set the LED pulse width (the IR, Red, and Green have the same
 * pulse width), and therefore, indirectly sets the integration time of the ADC
 * in each sample. The ADC resolution is directly related to the integration
 * time.
 *
 */
struct LED_PW : Field<SpO2Configuration, 1, 0> {
  /**
   * @brief PULSE WIDTH (µs)
   *
   */
  static constexpr float PulseWidth[] = {68.95, 117.78, 215.44, 410.75};
  static constexpr std::array<uint16_t, 4> PulseWidthInt{69, 118, 215, 411};

  /**
   * @brief ADC RESOLUTION (bits)
   *
   */
  static constexpr int AdcResolution[] = {15, 16, 17, 18};

  LED_PW(Field::REG &reg) : Field(reg) {}

  LED_PW &operator=(uint16_t pulseWidth) {
    const auto PulseWidthInt = LED_PW::PulseWidthInt;
    Field::operator=(
        details::nextGratherOrEqualIndex(PulseWidthInt, pulseWidth));
    return *this;
  }
};

template <uint8_t _REG> struct PARegister : Register<_REG> {

  /**
   * @brief TYPICAL LED CURRENT (mA)*
   *
   * Default is 0x1F which gets us 6.4mA
   * powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
   * powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
   * powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
   * powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch
   */

  static constexpr float typicalLedCurrent(uint8_t value) {
    return 50.f * 0xff / value;
  } // namespace registers

  PARegister &operator=(uint8_t value) {
    Register<_REG>::operator=(value);
    return *this;
  }
}; // namespace max30105

using LED1_PA = PARegister<0x0c>;
using LED2_PA = PARegister<0x0d>;
using LED3_PA = PARegister<0x0e>;

/**
 * @brief The purpose of PILOT_PA[7:0] is to set the LED power during the
 * proximity mode, as well as in Multi-LED mode.
 *
 */
using PILOT_PA = PARegister<0x10>;

/**
 * In multi-LED mode, each sample is split into up to four time slots, SLOT1
 * through SLOT4. These control registers determine which LED is active in each
 * time slot, making for a very flexible configuration.
 *
 */

template <typename _REG, uint8_t _LAST_BIT, uint8_t _FIRST_BIT>
struct Slot : Field<_REG, _LAST_BIT, _FIRST_BIT> {
  enum Led : uint8_t {
    Disabled,
    Led1, ///< Red, LED1_PA
    Led2, ///< IR, LED2_PA
    Led3, ///< Green, LED3_PA
    None,
    Led1Pilot, ///< Red, PILOT_PA
    Led2Pilot, ///< IR, PILOT_PA
    Led3Pilot, ///< Green, PILOT_PA

    LedRed = Led1,
    LedIR = Led2,
    LedGreen = Led3,

    LedRedPilot = Led1Pilot,
    LedIRPilot = Led2Pilot,
    LedGreenPilot = Led3Pilot,
  };
  Slot(typename Field<_REG, _LAST_BIT, _FIRST_BIT>::REG &reg)
      : Field<_REG, _LAST_BIT, _FIRST_BIT>(reg) {}
  Slot &operator=(Led value) {
    Field<_REG, _LAST_BIT, _FIRST_BIT>::operator=(static_cast<uint8_t>(value));
    return *this;
  }
  operator Led() const { return static_cast<Led>(static_cast<uint8_t>(*this)); }
};

using MultiLedMode1 = Register<0x11>;
using SLOT2 = Slot<MultiLedMode1, 6, 4>;
using SLOT1 = Slot<MultiLedMode1, 2, 0>;
using MultiLedMode2 = Register<0x12>;
using SLOT4 = Slot<MultiLedMode2, 6, 4>;
using SLOT3 = Slot<MultiLedMode2, 2, 0>;

/**
 * @brief Temperature Integer
 *
 * The on-board temperature ADC output is split into two registers, one to store
 *the integer temperature and one to store the fraction. Both should be read
 *when reading the temperature data, and the equation below shows how to add the
 *two registers together: `TMEASURED = TINTEGER + TFRACTION` This register
 *stores the integer temperature data in 2’s complement format, where each bit
 *corresponds to 1°C.
 *
 */
struct TINT : Register<0x1F, Access::R> {
  static constexpr int8_t decode(uint8_t value) { return value; }
};

/**
 * @brief Temperature Fraction
 *
 * This register stores the fractional temperature data in increments of
 * 0.0625°C. If this fractional temperature is paired with a negative integer,
 * it still adds as a positive fractional value (e.g., -128°C + 0.5°C =
 * -127.5°C).
 *
 */
struct TFRAC : Field<Register<0x20, Access::R>, 3, 0> {
  static constexpr float decode(uint8_t value) { return 0.0625f * value; }
};

/**
 * @brief Temperature Enable (TEMP_EN)
 *
 * This is a self-clearing bit which, when set, initiates a single temperature
 * reading from the temperature sensor. This bit clears automatically back to
 * zero at the conclusion of the temperature reading when the bit is set to one
 * in particle sensing mode.
 *
 */
using TEMP_EN = Bit<Register<0x21, Access::R>, 0>;

/**
 * @brief Proximity Mode Interrupt Threshold
 *
 * This register sets the IR ADC count that will trigger the beginning of
 * particle-sensing mode. The threshold is defined as the 8 MSBs of the ADC
 * count. For example, if PROX_INT_THRESH[7:0] = 0x01, then an ADC value of 1023
 * (decimal) or higher triggers the PROX interrupt. If PROX_INT_THRESH[7:0] =
 * 0xFF, then only a saturated ADC triggers the interrupt.
 *
 */
using PROX_INT_THRESH = Register<0x30>;

using REV_ID = Register<0xFE, Access::R>;
using PART_ID = Register<0xFF, Access::R, 0x15>;
} // namespace max30105_registers