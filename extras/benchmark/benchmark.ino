//#define FIXED_POINT_MATH
//#define SINGLE_PRECISION_MATH
#define DOUBLE_PRECISION_MATH

#include "pid.h"
#ifdef FIXED_POINT_MATH
  const uint32_t setpoint = 25420;
  const int32_t kp = -1011120;
  const int32_t ki = -1320*1000;
  const int32_t kd = -5280 * 1000;
  const uint8_t qn = 20;
  PID pidController = PID(setpoint, kp, ki, kd, qn);
#endif    // End FIXED_POINT_MATH

#if defined(SINGLE_PRECISION_MATH) || defined(DOUBLE_PRECISION_MATH)
  #ifdef DOUBLE_PRECISION_MATH
  typedef double myFloatType;
  #elif defined(SINGLE_PRECISION_MATH)
  typedef float myFloatType;
  #endif

ProportionalGain proportionalGain = proportionalToError;
myFloatType previousInput = 0;
myFloatType outputSum = 0;
myFloatType setpoint = 24.00;
myFloatType kp = -383.00;
myFloatType ki = -100;
myFloatType kd = -2.0;

static inline const __attribute__((always_inline, unused)) myFloatType clamp(myFloatType value, myFloatType min, myFloatType max) {
    return (value < min) ? min : (value > max) ? max : value;
}

// This implementation was taken from the exellent Arduino PID library written by Brett Beauregard.
// It was adapted so allow single precision as well.
// https://github.com/br3ttb/Arduino-PID-Library
myFloatType __attribute__((noinline)) compute(const myFloatType input) {
    const myFloatType error = setpoint - input;
    const myFloatType dInput = (input - previousInput);
    outputSum+= (ki * error);

    if (UNLIKELY(proportionalGain == proportionalToInput)) {
        outputSum-= kp * dInput;
    }

    outputSum = clamp(outputSum, 0.00, 4055.00);

    myFloatType output = outputSum - kd * dInput;
    if (LIKELY(proportionalGain == proportionalToError)) {
        output += kp * error;
    }

    output = clamp(output, 0.00, 4055.00);

    previousInput = input;
    return output;
}
#endif    // End SINGLE_PRECISION_MATH || DOUBLE_PRECISION_MATH

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  #ifdef FIXED_POINT_MATH
    pidController.setOutputMin(0);
    pidController.setOutputMax(UINT32_MAX);
  #endif

  // Enable CPU cycle counter
  #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)    // Teensy 3.x (Cortex M4)
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  #endif
}

void loop() {
  while (Serial.available()) {
   Serial.read();
  }

  uint32_t output = 42;
  #ifdef FIXED_POINT_MATH
    uint32_t previousInput = 25420;
    pidController.init(previousInput);
  #endif
  #if defined(SINGLE_PRECISION_MATH) || defined(DOUBLE_PRECISION_MATH)
    previousInput = 24.00;
  #endif

  cli();
  #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)    // Teensy 3.x (Cortex M4)
    const uint32_t cpuCycleCountStart = ARM_DWT_CYCCNT;
  #else   // Teensy LC
    const uint32_t cpuCycleCountStop = SYST_CVR;  // The counter is counting downwards, so we swap start/stop
  #endif

  // The actual function under test
  #ifdef FIXED_POINT_MATH
    output = pidController.compute(25420+80);
  #elif defined(SINGLE_PRECISION_MATH) || defined(DOUBLE_PRECISION_MATH)
    output = compute(26.0);
  #endif
  
  #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)    // Teensy 3.x (Cortex M4)
    const uint32_t cpuCycleCountStop = ARM_DWT_CYCCNT;
  #else   // Teensy LC
    const uint32_t cpuCycleCountStart = SYST_CVR; // The counter is counting downwards, so we swap start/stop
  #endif
  sei();
  Serial.print("CPU cycles per call: ");
  Serial.println(cpuCycleCountStop - cpuCycleCountStart);
  Serial.print("output: ");
  Serial.println(output);
  delay(500);
}
