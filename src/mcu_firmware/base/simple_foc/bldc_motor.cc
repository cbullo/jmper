#include "bldc_motor.h"

#include "default_configuration.h"
#include "hardware_api.h"
#include "sin_approx.h"
#include "time_utils.h"

#define N_SIN 4096
#define N_SIN_4 1024
#define N_SIN_3 1365
#define N_SIN_2_3 2731

inline uint16_t normalizeAngle(int32_t in, uint16_t period) {
  while (in < 0) {
    in += period;
  }

  while (in >= period) {
    in -= period;
  }

  return in;
}

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
BLDCMotor::BLDCMotor(CustomMagneticSensorI2C* s, int phA, int phB, int phC,
                     uint8_t pp, int8_t dir) {
  sensor = s;

  // save pole pairs number
  pole_pairs = pp;

  sensor_direction = dir;

  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  // enableA_pin = en1;
  // enableB_pin = en2;
  // enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = VoltsToVoltage(28) - 1;

  // maximum voltage to be set to the motor
  // voltage_limit = VoltsToVoltage(DEF_POWER_SUPPLY);

  // default target value
  // target = 0;
  voltage = 0;
}

// disable motor driver
void BLDCMotor::disable() {
  // set zero to PWM
  setPwm(0, 0, 0);
  // motor status update
  enabled = 0;
}
// enable motor driver
void BLDCMotor::enable() {
  // enableDriver();
  //  set zero to PWM
  setPwm(0, 0, 0);
  // motor status update
  enabled = 1;
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void BLDCMotor::loopFOC() {
  sensor->update();

  // if disabled do nothing
  if (!enabled) {
    return;
  }

  if (controller == MotionControlType::voltage) {
    electrical_angle = electricalAngle();
    // set the phase voltage - FOC heart function :)
    int16_t offset = N_SIN_4;
    if (voltage < 0) {
      offset = -offset;
    }
    Angle angle_to_set = normalizeAngle(electrical_angle + offset, N_SIN);
    setPhaseVoltageSin2(voltage, angle_to_set);
  } else if (controller == MotionControlType::direct_phase) {
    setPhaseVoltageSin2(voltage, phase);
  }

  // setPhaseVoltageSin2(5 * 512, phase);
  // phase++;
  // _delay(2);
}

void BLDCMotor::setPhaseVoltageSin2(Voltage U, Angle angle_el) {
  int32_t pwm_a;
  int32_t pwm_b;
  int32_t pwm_c;

  if (U > MAX_VOLTAGE * 512) {
    U = MAX_VOLTAGE * 512;
  } else if (U < -MAX_VOLTAGE * 512) {
    U = -MAX_VOLTAGE * 512;
  }

  int angle = angle_el;
  int16_t signed_angle = angle;
  if (angle > 2048) {
    signed_angle = 2048 - angle;
  }
  signed_angle *= (1 << 3);
  pwm_a = static_cast<int32_t>(sin15(signed_angle)) + (1 << 12) + 1;

  angle = normalizeAngle(angle_el + N_SIN_3, N_SIN);
  signed_angle = angle;
  if (angle > 2048) {
    signed_angle = 2048 - angle;
  }
  signed_angle *= (1 << 3);
  pwm_b = static_cast<int32_t>(sin15(signed_angle)) + (1 << 12) + 1;

  angle = normalizeAngle(angle_el + N_SIN_2_3, N_SIN);
  signed_angle = angle;
  if (angle > 2048) {
    signed_angle = 2048 - angle;
  }
  signed_angle *= (1 << 3);
  pwm_c = static_cast<int32_t>(sin15(signed_angle)) + (1 << 12) + 1;

  // auto minv = std::min(pwm_a, std::min(pwm_b, pwm_c));
  // pwm_a -= minv;
  // pwm_b -= minv;
  // pwm_c -= minv;

  int16_t power = abs(U);

  // apply power factor
  uint32_t pwm_a_u = power * pwm_a;

  pwm_a_u /= MAX_VOLTAGE * 514;
  pwm_a_u = pwm_a_u >> 5;
  // pwm_a += center;

  uint32_t pwm_b_u = power * pwm_b;

  pwm_b_u /= MAX_VOLTAGE * 514;
  pwm_b_u = pwm_b_u >> 5;
  // pwm_b += center;

  uint32_t pwm_c_u = power * pwm_c;

  pwm_c_u /= MAX_VOLTAGE * 514;
  pwm_c_u = pwm_c_u >> 5;
  // pwm_c += center;

  // pwm_a = std::max(0, std::min(pwm_a, 12 * 512));
  // pwm_b = std::max(0, std::min(pwm_b, 12 * 512));
  // pwm_c = std::max(0, std::min(pwm_c, 12 * 512));

  // uint16_t vps = 8 * 512;
  // vps >>= 9;

  // uint16_t uua = pwm_a;
  // uint16_t uub = pwm_b;
  // uint16_t uuc = pwm_c;

  uint8_t Ua = pwm_a_u;
  uint8_t Ub = pwm_b_u;
  uint8_t Uc = pwm_c_u;
  setPwm(pwm_a_u, pwm_b_u, pwm_c_u);
}

void BLDCMotor::setPhaseVoltageSin(Voltage U, Angle angle_el) {
  // Sinusoidal PWM modulation
  // Inverse Park + Clarke transformation

  // angle_el = normalizeAngle(angle_el + 1024, 4096);

  // angle normalization in between 0 and 2pi
  // only necessary if using _sin and _cos - approximation functions
  // angle_el = normalizeAngle(angle_el, 4096);
  int32_t _ca = sin15(normalizeAngle(1024 - angle_el, 4096) << 3);
  int32_t _sa = sin15(static_cast<uint16_t>(angle_el) << 3);
  // Inverse park transform
  int32_t Ualpha = -_sa * U / (1 << 9);
  int32_t Ubeta = _ca * U / (1 << 9);

  // center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
  int32_t center = voltage_limit / 2;
  // Clarke transform
  Ua = Ualpha + center;
  Ub = -Ualpha / 2 + (443 * Ubeta) / 512 + center;
  Uc = -Ualpha / 2 - (443 * Ubeta) / 512 + center;

  // setPwm(Ua, Ub, Uc);

  // setPwm(0, 3 * 512, 0);
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usaage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void BLDCMotor::setPhaseVoltage(Voltage U, Angle angle_el) {
  int16_t sectors[6] = {683, 1365, 2048, 2731, 3413, 4095};

  int32_t center;
  int16_t _ca, _sa;

  uint16_t sqrt3 = 887;  // 2^9

  // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
  // https://www.youtube.com/watch?v=QMSWUMEAejg

  // the algorithm goes
  // 1) Ualpha, Ubeta
  // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
  // 3) angle_el = atan2(Ubeta, Ualpha)
  //
  // equivalent to 2)  because the magnitude does not change is:
  // Uout = sqrt(Ud^2 + Uq^2)
  // equivalent to 3) is
  // angle_el = angle_el + atan2(Uq,Ud)

  // U /= 256;

  int16_t Uout = U;  // 2^9
  // a bit of optitmisation
  {  // only Uq available - no need for atan2 and sqrt
    // Uout = U / driver->voltage_limit;
    // angle normalisation in between 0 and 2pi
    // only necessary if using _sin and _cos - approximation functions
    angle_el = normalizeAngle(angle_el + 1024, 4096);
  }

  // calculate the duty cycles(times)
  int32_t Ta, Tb, Tc;
  if (angle_el <= sectors[0]) {
    int16_t s1 = sin15((sectors[0] - angle_el) << 3);
    int16_t s2 = sin15(angle_el << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
  } else if (angle_el <= sectors[1]) {
    int16_t s1 = sin15((sectors[1] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[0]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;

  } else if (angle_el <= sectors[2]) {
    int16_t s1 = sin15((sectors[2] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[1]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
  } else if (angle_el <= sectors[3]) {
    int16_t s1 = sin15((sectors[3] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[2]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
  } else if (angle_el <= sectors[4]) {
    int16_t s1 = sin15((sectors[4] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[3]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }

    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
  } else if (angle_el <= sectors[5]) {
    int16_t s1 = sin15((sectors[5] - angle_el) << 3);
    int16_t s2 = sin15((angle_el - sectors[4]) << 3);

    int32_t sqrt3s1 = (sqrt3 * s1) / (1 << 9);
    int32_t sqrt3s2 = (sqrt3 * s2) / (1 << 9);

    int32_t T1 = ((sqrt3s1 * Uout) / voltage_limit);
    int32_t T2 = ((sqrt3s2 * Uout) / voltage_limit);
    // two versions possible
    int32_t T0 = 0;  // pulled to 0 - better for low power supply voltage
    if (true) {
      // modulation_centered around driver->voltage_limit/2
      T0 = (1 << 9) - T1 - T2;
    }

    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
  } else {
    // possible error state
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  // calculate the phase voltages and center
  Ua = Ta * voltage_limit / (1 << 9);
  Ub = Tb * voltage_limit / (1 << 9);
  Uc = Tc * voltage_limit / (1 << 9);

  // set the voltages in driver
  //setPwm(Ua, Ub, Uc);
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
// uint16_t BLDCMotor::velocityOpenloop(Velocity target_velocity) {
//   // get current timestamp
//   unsigned long now_us = _micros();
//   // calculate the sample time from last call
//   float Ts = (now_us - open_loop_timestamp) * 1e-6f;
//   // quick fix for strange cases (micros overflow + timestamp not defined)
//   if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

//   // calculate the necessary angle to achieve target velocity
//   shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
//   // for display purposes
//   shaft_velocity = target_velocity;

//   // use voltage limit or current limit
//   float Uq = voltage_limit;

//   // set the maximal allowed voltage (voltage_limit) with the necessary angle
//   setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));

//   // save timestamp for next call
//   open_loop_timestamp = now_us;

//   return Uq;
// }

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
// uint16_t BLDCMotor::angleOpenloop(LargeAngle target) {
//   // get current timestamp
//   unsigned long now_us = _micros();
//   // calculate the sample time from last call
//   float Ts = (now_us - open_loop_timestamp) * 1e-6f;
//   // quick fix for strange cases (micros overflow + timestamp not defined)
//   if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

//   // calculate the necessary angle to move from current position towards
//   target
//   // angle with maximal velocity (velocity_limit)
//   // TODO sensor precision: this calculation is not numerically precise. The
//   // angle can grow to the point
//   //                        where small position changes are no longer
//   captured
//   //                        by the precision of floats when the total
//   position
//   //                        is large.
//   if (abs(target_angle - shaft_angle) > abs(velocity_limit * Ts)) {
//     shaft_angle += _sign(target_angle - shaft_angle) * abs(velocity_limit) *
//     Ts; shaft_velocity = velocity_limit;
//   } else {
//     shaft_angle = target_angle;
//     shaft_velocity = 0;
//   }

//   // use voltage limit or current limit
//   float Uq = voltage_limit;
//   if (_isset(phase_resistance)) Uq = current_limit * phase_resistance;
//   // set the maximal allowed voltage (voltage_limit) with the necessary angle
//   // sensor precision: this calculation is OK due to the normalisation
//   setPhaseVoltage(Uq, 0,
//                   _electricalAngle(_normalizeAngle(shaft_angle),
//                   pole_pairs));

//   // save timestamp for next call
//   open_loop_timestamp = now_us;

//   return Uq;
// }

// init hardware pins
int BLDCMotor::init() {
  digitalWrite(pwmA, LOW);
  digitalWrite(pwmB, LOW);
  digitalWrite(pwmC, LOW);

  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  // digitalWrite(pwmA, LOW);
  // digitalWrite(pwmB, LOW);
  // digitalWrite(pwmC, LOW);
  // if (_isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);
  // if (_isset(enableB_pin)) pinMode(enableB_pin, OUTPUT);
  // if (_isset(enableC_pin)) pinMode(enableC_pin, OUTPUT);

  // sanity check for the voltage limit configuration
  if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
    voltage_limit = voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);

  //_delay(500);
  // enable motor
  disable();
  //_delay(500);
  return 0;
}

// // Set voltage to the pwm pin
// void BLDCMotor::setPhaseState(int sa, int sb, int sc) {
//   // disable if needed
//   if (_isset(enableA_pin) && _isset(enableB_pin) && _isset(enableC_pin)) {
//     digitalWrite(enableA_pin, sa == _HIGH_IMPEDANCE ? LOW : HIGH);
//     digitalWrite(enableB_pin, sb == _HIGH_IMPEDANCE ? LOW : HIGH);
//     digitalWrite(enableC_pin, sc == _HIGH_IMPEDANCE ? LOW : HIGH);
//   }
// }

uint8_t dc_value_00 = 0;
uint8_t dc_value_01 = 0;
uint8_t dc_value_02 = 0;
uint8_t dc_value_10 = 0;
uint8_t dc_value_11 = 0;
uint8_t dc_value_12 = 0;

extern uint8_t motor_pin_00;

extern bool disable_motors_temp_read;

// Set voltage to the pwm pin
void BLDCMotor::setPwm(uint8_t dc_a, uint8_t dc_b, uint8_t dc_c) {
  // limit the voltage in driver
  // Ua = max(0, min(Ua, voltage_limit));
  // Ub = max(0, min(Ub, voltage_limit));
  // Uc = max(0, min(Uc, voltage_limit));

  // uint16_t vps = voltage_power_supply;
  // vps >>= 8;

  // uint16_t uua = Ua;
  // uint16_t uub = Ub;
  // uint16_t uuc = Uc;

  // dc_a = uua / (2 * (voltage_power_supply >> 9));
  // dc_b = uub / (2 * (voltage_power_supply >> 9));
  // dc_c = uuc / (2 * (voltage_power_supply >> 9));

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  //_writeDutyCycle3PWM(dc_a, dc_b, dc_c, pwmA, pwmB, pwmC);

  // if (pwmA == motor_pin_00) {
  //   dc_value_00 = dc_a;
  //   dc_value_01 = dc_b;
  //   dc_value_02 = dc_c;
  // } else {
  //   dc_value_10 = dc_a;
  //   dc_value_11 = dc_b;
  //   dc_value_12 = dc_c;
  // }

  if (!disable_motors_temp_read) {
    _writeDutyCycle3PWM(dc_a, dc_b, dc_c, pwmA, pwmB, pwmC);
  }
}

// // shaft angle calculation
// Angle BLDCMotor::shaftAngle() {
//   // if no sensor linked return previous value ( for open loop )
//   if (!sensor) return shaft_angle;
//   return sensor_direction * sensor->getAngle() - sensor_offset;
// }

Angle BLDCMotor::electricalAngle() {
  // if no sensor linked return previous value ( for open loop )

  int32_t pos = sensor->getMechanicalAngle();

  return normalizeAngle(
      (sensor_direction * pole_pairs) * (zero_electric_angle - pos), 4096);

  // pos *= pole_pairs;
  // pos -= static_cast<int32_t>(zero_electric_angle) * pole_pairs;

  // uint16_t normalized_pos = normalizeAngle(pos, 4096);
  // return normalized_pos;
  // // return normalizeAngle(sensor_direction *
  // //                       static_cast<int16_t>(normalized_pos), 4096);
}

// float FOCMotor::electricalAngle(){
//   // if no sensor linked return previous value ( for open loop )
//   if(!sensor) return electrical_angle;
//   return  _normalizeAngle( (float)(sensor_direction * pole_pairs) *
//   sensor->getMechanicalAngle()  - zero_electric_angle );
// }

void BLDCMotor::linkSensor(CustomMagneticSensorI2C* _sensor) {
  sensor = _sensor;
}