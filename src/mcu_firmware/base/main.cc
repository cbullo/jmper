
// #include "../../custom_magnetic_sensor_i2c.h"
#include <avr/wdt.h>

#include "Arduino.h"
// #include "SimpleFOC.h"
#include "Wire.h"
#include "analog_reader.h"
#include "debug_led.h"
#include "simple_foc/time_utils.h"
#include "temperature.h"

#define DEBUG_WDT 0

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t
// _angle_register_msb)
//  chip_address  I2C chip address
//  bit_resolution  resolution of the sensor
//  angle_register_msb  angle read register msb
//  bits_used_msb  number of used bits in msb register
//
// make sure to read the chip address and the chip angle register msb value from
// the datasheet also in most cases you will need external pull-ups on SDA and
// SCL lines!!!!!
//
// For AS5058B
// MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// Example of AS5600 configuration
// CustomMagneticSensorI2C sensor = CustomMagneticSensorI2C(AS5600_I2C, A1, A0);
AnalogReader analog_reader = AnalogReader();
unsigned long next_temperature_read;

void Initialize();
void Tick();
void Critical();

// uint8_t resetFlag __attribute__((section(".noinit")));
// void resetFlagsInit(void) __attribute__((naked)) __attribute__((used))
// __attribute__((section(".init0")));
// void resetFlagsInit(void) {
//   /*
//      save the reset flags passed from the bootloader
//      This is a "simple" matter of storing (STS) r2 in the special variable
//      that we have created.  We use assembler to access the right variable.
//   */
//   __asm__ __volatile__("sts %0, r2\n" : "=m"(resetFlag) :);
// }

void MainInitialize() {
  LEDPIN_PINMODE
  LEDPIN_OFF

  wdt_disable();
  Serial.begin(115200);
  // while (!Serial)
  //  ;

  // Serial.println("Reset reason:");
  // Serial.println(resetFlag);
  // if (resetFlag & _BV(EXTRF)) {
  //   // Reset button or otherwise some software reset
  //   Serial.println("Reset button was pressed.");
  // }
  // if (resetFlag & (_BV(BORF) | _BV(PORF))) {
  //   // Brownout or Power On
  //   Serial.println("Power loss occured!");
  // }
  // if (resetFlag & _BV(WDRF)) {
  //   // Watchdog Reset
  //   Serial.println("Watchdog Reset");
  // }

  // configure i2C
  Wire.setClock(400000);

  analog_reader.StartConversion(2, 0);
  // Serial.println(F("Done setup"));
  wdt_enable(WDTO_4S);
  next_temperature_read = _micros();
  Initialize();
}

const uint8_t buckets_offset = 10;
const uint8_t buckets_count = 100;
const uint8_t buffer_size = 32;
uint16_t values_in_buckets[2] = {};

uint8_t temp_values_buckets[2][buckets_count] = {};  // 10 to 110 degrees C
uint8_t temp_ring_buffer[2][buffer_size] = {};
uint8_t buffer_pos[2] = {};

uint8_t UpdateAndGetMedian(uint8_t index, uint8_t new_value) {
  if (buckets_offset <= new_value &&
      new_value < buckets_offset + buckets_count) {
    new_value -= buckets_offset;

    auto old_value = temp_ring_buffer[index][buffer_pos[index]];
    if (buckets_offset <= old_value &&
        old_value < buckets_offset + buckets_count) {
      old_value -= buckets_offset;
      --temp_values_buckets[index][old_value];
      --values_in_buckets[index];
    }
    ++temp_values_buckets[index][new_value];
    ++values_in_buckets[index];
    temp_ring_buffer[index][buffer_pos[index]] = new_value + buckets_offset;
    buffer_pos[index] = (buffer_pos[index] + 1) % buffer_size;
  }

  if (values_in_buckets[index] < 4) {
    return 0;
  }

  uint16_t values_counter = 0;
  uint8_t value = 0;
  while (values_counter < (values_in_buckets[index] / 2) &&
         value < buckets_count) {
    values_counter += temp_values_buckets[index][value];
    value++;
  }
  return value - 1 + buckets_offset;
}

uint16_t temperature[2] = {20, 20};
void CheckTemperature(bool first) {
  uint8_t index = first ? 1 : 0;
  int reading = analog_reader.GetADCReading(index);

  uint8_t temp_reading = ComputeTemperature(reading);
  // uint16_t temp_8 = temperature[index] >> 3;


  uint8_t median_temp = UpdateAndGetMedian(index, temp_reading);
  temperature[index] = median_temp;

  // temperature[index] -= temp_8;
  // temperature[index] += temp_reading >> 3;

  if (temperature[index] > (55)) {
    // Serial.print(F("temperature: "));
    // Serial.println(temperature[index]);
    // Serial.println(F("OVERHEAT DETECTED! RESETTING!"));
    wdt_enable(WDTO_15MS);
    _delay(100000);  // This will trigger WDT reset
  }
  analog_reader.StartConversion(first ? 2 : 3, 1 - index);
}

void serialEventRun(){};

extern uint8_t motor_pin_00;
extern uint8_t motor_pin_01;
extern uint8_t motor_pin_02;
extern uint8_t motor_pin_10;
extern uint8_t motor_pin_11;
extern uint8_t motor_pin_12;

extern uint8_t dc_value_00;
extern uint8_t dc_value_01;
extern uint8_t dc_value_02;
extern uint8_t dc_value_10;
extern uint8_t dc_value_11;
extern uint8_t dc_value_12;

bool disable_motors_temp_read = false;

#define TEMP_CHECK_PERIOD 100000;  // us
void MainCritical() {
  auto us = _micros();

  if (us >= next_temperature_read) {
    bool first = next_temperature_read % 200000 < 100000;
    // uint8_t t_index = first ? 0 : 1;
    // Add all safety/hardware failure critical functions here
    CheckTemperature(first);
    next_temperature_read = us + TEMP_CHECK_PERIOD;
  }
  Critical();

  // This should be the only place where WDT is reset!
  wdt_reset();
}

#if DEBUG_WDT
ISR(WDT_vect) { LEDPIN_ON }
#endif

int main() {
  MCUSR = 0;
  init();
  MainInitialize();

#if DEBUG_WDT
  cli();
  WDTCSR |= _BV(WDIE);
  sei();
#endif

  while (true) {
    Tick();
    MainCritical();
  }
  return 0;
}