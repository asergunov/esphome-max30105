/// Datasheet
/// https://cdn.sparkfun.com/assets/learn_tutorials/5/7/7/MAX30105_3.pdf
#pragma once

#include "esphome/components/max30105/max30105_registers.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <stdint.h>
#include <tuple>

namespace max30105_registers {
namespace details {
template <typename T, size_t N>
constexpr size_t nextGreaterOrEqualIndex(const std::array<T, N> &list,
                                         const T &value) {

  return std::min(1, std::distance(std::begin(list),
                                   std::upper_bound(std::begin(list),
                                                    std::end(list), value))) -
         1;
}

// Tuple visiting
template <int... Is> struct seq {};

template <int N, int... Is> struct gen_seq : gen_seq<N - 1, N - 1, Is...> {};

template <int... Is> struct gen_seq<0, Is...> : seq<Is...> {};

template <typename T, typename F, int... Is>
void for_each(T &&t, F f, seq<Is...>) {
  auto l = {(f(std::get<Is>(t)), 0)...};
}
} // namespace details

template <typename... Ts, typename F>
void for_each_in_tuple(std::tuple<Ts...> const &t, F f) {
  details::for_each(t, f, details::gen_seq<sizeof...(Ts)>());
}

enum class Access { R, W, RW };

template <typename _REG, uint8_t _LAST_BIT = 7, uint8_t _FIRST_BIT = 0>
struct Field;

template <uint8_t _REG_ADR, Access _ACCESS = Access::RW,
          uint8_t _POR_STATE = 0x00>
struct Register {
  static constexpr auto REG_ADR = _REG_ADR;
  static constexpr auto POR_STATE = _POR_STATE;

  explicit constexpr Register(uint8_t val) : value(val) {}
  constexpr Register() {}

  template <uint8_t _LAST_BIT, uint8_t _FIRST_BIT>
  Register &operator<<(const Field<Register, _LAST_BIT, _FIRST_BIT> &field) {
    using Field = Field<Register, _LAST_BIT, _FIRST_BIT>;
    value = (value & ~Field::MASK) |
            (Field::MASK & (static_cast<uint8_t>(field) << Field::BEGIN_BIT));
    return *this;
  }

  template <uint8_t _LAST_BIT, uint8_t _FIRST_BIT>
  const Register &
  operator>>(Field<Register, _LAST_BIT, _FIRST_BIT> &field) const {
    using Field = Field<Register, _LAST_BIT, _FIRST_BIT>;
    field = Field((value >> Field::BEGIN_BIT) & Field::MASK);
    return *this;
  }

  constexpr operator const uint8_t &() const { return value; }
  operator uint8_t &() { return value; }

  Register operator=(const uint8_t &raw) {
    value = raw;
    return *this;
  }

private:
  uint8_t value = POR_STATE;
};

template <typename _REG, uint8_t _LAST_BIT, uint8_t _FIRST_BIT> struct Field {
  static constexpr auto BEGIN_BIT = _FIRST_BIT;
  static constexpr auto END_BIT = _LAST_BIT + 1;
  static constexpr auto MASK = ((1u << END_BIT) - 1u) << BEGIN_BIT;
  using REG = _REG;

  Field() {}
  constexpr Field(const uint8_t &value) : _value(value) {}
  // constexpr Field(const REG &reg) : Field(static_cast<Field>(reg)) {}
  constexpr operator const uint8_t &() const { return _value; }

private:
  uint8_t _value;
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
struct SMP_AVE : Field<FIFOConfiguration, 7, 5> {
  using Base = Field<FIFOConfiguration, 7, 5>;
  SMP_AVE() {}
  explicit SMP_AVE(uint8_t average)
      : Base(average >= 32   ? 0b101
             : average >= 16 ? 0b100
             : average >= 8  ? 0b011
             : average >= 4  ? 0b010
             : average > 2   ? 0x001
                             : 0x00) {}
  explicit operator uint8_t() const {
    auto value = Base::operator const uint8_t &();
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

enum class Mode {
  None = 0x000,
  ParticleSensing1LED = 0b010,
  ParticleSensing2LED = 0b011,
  MultiLed = 0b111,
};
/**
 * @brief Mode Control
 *
 * These bits set the operating state of the MAX30105. Changing modes does not
 * change any other setting, nor does it erase any previously stored data inside
 * the data registers.
 *
 */
struct MODE : Field<ModeConfiguration, 2, 0> {
  MODE(Mode mode = Mode::None) : Field(static_cast<uint8_t>(mode)) {}

  operator Mode() const {
    const auto result = static_cast<Mode>(static_cast<uint8_t>(*this));
    switch (result) {
    case Mode::ParticleSensing1LED:
    case Mode::ParticleSensing2LED:
    case Mode::MultiLed:
      return result;
    default:
      return Mode::None;
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
  static constexpr std::array<float, 4> LsbSize{7.81, 15.63, 31.25, 62.5};

  /**
   * @brief FULL SCALE (nA)
   *
   */
  static constexpr std::array<uint16_t, 4> FullScale{2048, 4096, 8192, 16384};

  static ADC_RGE fromLsbSize(const float &lsbSize) {
    return ADC_RGE(details::nextGreaterOrEqualIndex(LsbSize, lsbSize));
  }

  static ADC_RGE fromFulScale(const uint16_t &fullScale) {
    return ADC_RGE(details::nextGreaterOrEqualIndex(FullScale, fullScale));
  }

private:
  explicit ADC_RGE(uint8_t value) : Field(value) {}
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

  SR(uint16_t samplesPerSecond)
      : Field([&] {
          return details::nextGreaterOrEqualIndex(SamplesPerSecond,
                                                  samplesPerSecond);
        }()) {}
  operator uint16_t() const {
    return SamplesPerSecond[static_cast<uint8_t>(
        static_cast<const Field &>(*this))];
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
  static constexpr std::array<float, 4> PulseWidth{68.95, 117.78, 215.44,
                                                   410.75};
  static constexpr std::array<uint16_t, 4> PulseWidthInt{69, 118, 215, 411};

  /**
   * @brief ADC RESOLUTION (bits)
   *
   */
  static constexpr std::array<uint8_t, 4> AdcResolution = {15, 16, 17, 18};

  static constexpr LED_PW fromPulseWidth(float pulseWidth) {
    return LED_PW(details::nextGreaterOrEqualIndex(PulseWidth, pulseWidth));
  }

  static constexpr LED_PW fromPulseWidth(uint16_t pulseWidth) {
    return LED_PW(details::nextGreaterOrEqualIndex(PulseWidthInt, pulseWidth));
  }

  static constexpr LED_PW fromAdcResolution(uint8_t adcResolution) {
    return LED_PW(
        details::nextGreaterOrEqualIndex(AdcResolution, adcResolution));
  }

private:
  explicit constexpr LED_PW(uint8_t raw) : Field(raw) {}
};

template <typename _REG> struct PAField : Field<_REG> {
  using Base = Field<_REG>;
  /**
   * @brief TYPICAL LED CURRENT (mA)*
   *
   * Default is 0x1F which gets us 6.4mA
   * powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
   * powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
   * powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
   * powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch
   */

  static constexpr float typicalLedCurrent(uint8_t raw) {
    return 50.f * raw / 0xff;
  } // namespace registers

  static constexpr PAField fromLedCurrent(float value) {
    return PAField(std::max(0, std::min(0xff, int(value / 50.0 * 0xff))));
  }
  explicit constexpr PAField(uint8_t raw) : Base(raw) {}
}; // namespace max30105

using LED1_PA = PAField<Register<0x0c>>;
using LED2_PA = PAField<Register<0x0d>>;
using LED3_PA = PAField<Register<0x0e>>;

/**
 * @brief The purpose of PILOT_PA[7:0] is to set the LED power during the
 * proximity mode, as well as in Multi-LED mode.
 *
 */
using PILOT_PA = PAField<Register<0x10>>;

enum class Slot : uint8_t {
  None,
  Led1, ///< Red, LED1_PA
  Led2, ///< IR, LED2_PA
  Led3, ///< Green, LED3_PA
  NonePilot,

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

/**
 * In multi-LED mode, each sample is split into up to four time slots, SLOT1
 * through SLOT4. These control registers determine which LED is active in each
 * time slot, making for a very flexible configuration.
 *
 */

template <typename _REG, uint8_t _LAST_BIT, uint8_t _FIRST_BIT>
struct SlotField : Field<_REG, _LAST_BIT, _FIRST_BIT> {
  using Base = Field<_REG, _LAST_BIT, _FIRST_BIT>;
  SlotField(Slot slot = Slot::None) : Base(static_cast<uint8_t>(slot)) {}
  operator Slot() const {
    const auto &value = static_cast<uint8_t>(*static_cast<const Base &>(*this));
    return value <= 0b111 ? static_cast<Slot>(value) : Slot::None;
  }
};

using MultiLedMode1 = Register<0x11>;
using SLOT2 = SlotField<MultiLedMode1, 6, 4>;
using SLOT1 = SlotField<MultiLedMode1, 2, 0>;
using MultiLedMode2 = Register<0x12>;
using SLOT4 = SlotField<MultiLedMode2, 6, 4>;
using SLOT3 = SlotField<MultiLedMode2, 2, 0>;

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
struct TINT : Field<Register<0x1F, Access::R>> {
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
using PROX_INT_THRESH = Field<Register<0x30>>;

using REV_ID = Register<0xFE, Access::R>;
using PART_ID = Register<0xFF, Access::R, 0x15>;

struct Configuration
    : std::tuple<InterruptEnable1, InterruptEnable2, ModeConfiguration,
                 FIFOConfiguration, SpO2Configuration, LED1_PA::REG,
                 LED2_PA::REG, LED3_PA::REG, PILOT_PA::REG, MultiLedMode1,
                 MultiLedMode2, FIFO_RD_PTR::REG, FIFO_WR_PTR::REG,
                 OVF_COUNTER::REG> {

  template <typename _REG> const _REG &reg() const noexcept {
    return std::get<_REG>(*this);
  }
  template <typename _REG> _REG &reg() noexcept {
    return std::get<_REG>(*this);
  }

  template <typename _FIELD> _FIELD field() const noexcept {
    _FIELD field;
    reg<typename _FIELD::REG>() >> field;
    return field;
  }

  template <typename _REG, uint8_t _LAST_BIT, uint8_t _FIRST_BIT>
  Configuration &
  operator<<(const Field<_REG, _LAST_BIT, _FIRST_BIT> &field) noexcept {
    reg<_REG>() << field;
    return *this;
  }

  std::array<Slot, 4> ledSlots() {
    const auto &mode = field<MODE>();
    switch (static_cast<Mode>(mode)) {
    case Mode::ParticleSensing1LED:
      return {Slot::LedRed, Slot::Disabled, Slot::Disabled, Slot::Disabled};
    case Mode::ParticleSensing2LED:
      return {Slot::LedRed, Slot::LedIR, Slot::Disabled, Slot::Disabled};
    case Mode::MultiLed:
      return {field<SLOT1>(), field<SLOT2>(), field<SLOT3>(), field<SLOT4>()};
    }
    return {Slot::Disabled, Slot::Disabled, Slot::Disabled, Slot::Disabled};
  }
};

template <typename _REG> struct RegTraits {
  static constexpr auto name = [] {
    if constexpr (std::is_same_v<_REG, InterruptStatus1>)
      return "Interrupt Status 1";
    else if constexpr (std::is_same_v<_REG, InterruptStatus2>) {
      return "Interrupt Status 2";
    } else if constexpr (std::is_same_v<_REG, InterruptEnable1>) {
      return "Interrupt Enable 1";
    } else if constexpr (std::is_same_v<_REG, InterruptEnable2>) {
      return "Interrupt Enable 2";
    } else if constexpr (std::is_same_v<_REG, PART_ID>) {
      return "Part ID";
    } else if constexpr (std::is_same_v<_REG, PART_ID>) {
      return "Revision ID";
    } else if constexpr (std::is_same_v<_REG, OVF_COUNTER::REG>) {
      return "FIFO Overflow Counter";
    } else if constexpr (std::is_same_v<_REG, FIFO_WR_PTR::REG>) {
      return "FIFO Write Pointer";
    } else if constexpr (std::is_same_v<_REG, FIFO_DATA>) {
      return "FIFO Data";
    } else if constexpr (std::is_same_v<_REG, FIFO_RD_PTR::REG>) {
      return "FIFO Read Pointer";
    } else if constexpr (std::is_same_v<_REG, FIFOConfiguration>) {
      return "FIFO Configuration";
    } else if constexpr (std::is_same_v<_REG, ModeConfiguration>) {
      return "Mode Configuration";
    } else if constexpr (std::is_same_v<_REG, SpO2Configuration>) {
      return "SpO2 Configuration";
    } else if constexpr (std::is_same_v<_REG, LED1_PA::REG>) {
      return "Led1 Pulse Amplitude";
    } else if constexpr (std::is_same_v<_REG, LED2_PA::REG>) {
      return "Led2 Pulse Amplitude";
    } else if constexpr (std::is_same_v<_REG, LED3_PA::REG>) {
      return "Led3 Pulse Amplitude";
    } else if constexpr (std::is_same_v<_REG, PILOT_PA::REG>) {
      return "Pilot Pulse Amplitude";
    } else if constexpr (std::is_same_v<_REG, MultiLedMode1>) {
      return "MultiLed Mode 1";
    } else if constexpr (std::is_same_v<_REG, MultiLedMode2>) {
      return "MultiLed Mode 2";
    } else if constexpr (std::is_same_v<_REG, TINT::REG>) {
      return "Temperature Integer";
    } else if constexpr (std::is_same_v<_REG, TFRAC::REG>) {
      return "Temperature Integer";
    } else if constexpr (std::is_same_v<_REG, TEMP_EN::REG>) {
      return "Temperature Enable";
    } else if constexpr (std::is_same_v<_REG, PROX_INT_THRESH::REG>) {
      return "Proximity Interrupt Threshold";
    }
    return "Unknown";
  }();
  static constexpr auto address = _REG::REG_ADR;
  static constexpr auto powerOnState = _REG::POR_STATE;
};

} // namespace max30105_registers