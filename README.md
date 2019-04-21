# PID-CPP
This a C++ library implementing a PID controller intended for microcontrollers. It is a fixed point math implementation that was optimized for performance on the ARM Cortex M4 processors. The library was inspired by the excellent [blog series](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/) written by Brett Beauregard and the PID library he put on Github (https://github.com/br3ttb/Arduino-PID-Library/).

**This library is about a [factor of 7-10](#performance-comparison) faster than the Arduino PID library!**

## Floating Point vs. Fixed Point Arithmetic

### Floating Point Arithmetic
The library written by Brett Beauregard and numerous other implementations that can be found on the web do the calculations using floating point arithmetic. The advantage of this approach is, that one needs to know little about the input and output of the system as these datatypes have a large dynamic range (due to the use of the exponent bits). These libraries can therefore do their calculations in real units, which make the whole system more intuitive for newcomers.

The downside is, that without a dedicated floating point unit, both single precision and especially double precision floats are slow and the majority of all microcontrollers does not have an FPU. Also single precision floats are limited by their 24 bit precision (+1 sign bit).

### Fixed Point Arithmetic
Fixed point arithmetic on the other hand is a more natural representation for the control loop. In real life most input data comes from components like an ADC or sensor, which outputs some binary value in its own system of units. The PID output will then be written to some DAC and is scaled to real units by the output stage. In any application that does not require huge amounts of dynamic range, the additional bits used by floats to scale the range (see the table below) can then be used for more precision.

The downside is, that we need saturating mathematical operations to make sure, that the calculations do not produce unexpected results. Some microcontrollers like the Cortex M4 support DSP instructions, which can do this in hardware.

The table below gives a summary of the different datatypes. Using single precision floats and ADC/DAC combinations with more than 12 bit is questionable because the system 'gain' will then be limited to (24 - DAC_resolution).

|Datatype|Width|Precision|Comments|
|--|--|--|--|
|Single Precision Float|32 bit|24 + 1 sign bits|Requires 32-bit FPU to be efficient|
|Double Precision Float|64 bit|53 + 1 sign bits|Requires 64-bit FPU to be efficient|
|Q31 Fixed Point|32 bit|31 + 1 sign bits|Requires 32-bit processor to be efficient|

## API
#### Constructor:
```c
   enum FeedbackDirection {
     feedbackPositive = 0,
     feedbackNegative = 1,
   };
   enum ProportionalGain {
     proportionalToInput = 0,
     proportionalToError = 1,
   };

   PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, uint8_t _qn, FeedbackDirection feedbackDirection, ProportionalGain proportionalGain);
   PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, uint8_t _qn, FeedbackDirection feedbackDirection);
   PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, uint8_t _qn);
```
___Arguments___
* `setpoint` [unsigned long] : The value in units of the input. It is the sensor value the controller will servo
* `kp` [long] : The factor of the proportional term. The sign will ignored, but can be set by the `feedbackDirection` parameter
* `ki` [long] : The factor of the integral term. This value must be scaled (ki * period), if the sampling period changes.
* `kd` [long] : The factor of the derivative term. This value must be scaled (ki / period), if the sampling period changes.
* `qn` [unsigned short] : The number of bits used to designate the fractional portion of the output.  
Set this number to be `32 - DAC_resolution`
* `feedbackDirection` [FeedbackDirection] : Choose whether a positive error causes a positve response (feedbackPositive) or a negative response(feedbackNegative).  
Default: `feedbackNegative`
* `proportionalGain` [ProportionalGain] : Choose whether kp acts on the error (proportionalToError) or the deviation from the initial input (proportionalToInput). See [here](http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/) for more info.  
Default: `proportionalToError`

### Methods
```c
   const uint32_t compute(uint32_t input);
```
This function is the PID routine. It calculates the output based on the input and previous inputs. It needs to be called in regular intervals. In contrast to other libraries, this is not handled internally, because it gives more flexibility to the user.

___Arguments___
* `value` [unsigned long] : The sensor input

___Returns___
* [unsigned long] : The PID output. It will be scaled using the `qn` parameter, so that it does **not** contain the fractional part. It can be directly fed to the output hardware.

### Setters
```c
   void init(const uint32_t initialInput);
```
Call this function after switching from manual control to PID control or when initializing the PID controller.

___Arguments___
* `value` [unsigned long] : This value initilizes the internal previous input variable used to calculate the derivative term

```c
   void updateOutput(const uint32_t value);
```
Call this function whenever the output was manually changed to make the PID controller aware of the change. The PID controller needs to know the previous output when switching from off to on, to make sure there is no bump in the output due to a discontinuity.

___Arguments___
* `value` [unsigned long] : This value initilizes the internal error sum of the integral term.

```c
   void setOutputMin(const uint32_t value);
   void setOutputMax(const uint32_t value);
```
___Arguments___
* `value` [unsigned long] : This value clamps the output to the the values chosen. Do not clamp the output afterwards, because using this function makes sure that the PID controller is aware of the limits and treats the integral term appropriately.

```c
   void setTunings(const int32_t kp, const int32_t ki, const int32_t kd, const ProportionalGain proportionalGain);
   void setTunings(const int32_t kp, const int32_t ki, const int32_t kd);
   void setSetpoint(const uint32_t value);
   void setControllerFeedback(const FeedbackDirection feedbackDirection);
```
___Arguments___

See [constructor](#constructor)

### Setters
```c
   const uint32_t getKp();
   const uint32_t getKi();
   const uint32_t getKd();
   const uint32_t getSetpoint();
```
Use these to query the internal state.

## Examples
This is an example, which runs on the Arduino platform. It is very simple and does not implement advanced features like setpoint changes, or a manual mode. It only implements regular calls to the `compute()` method.
```c
#include <pid.h>    // Use quotation marks if the library is in the same folder as the sketch
unsigned long pidPeriod = 500;
unsigned long setpoint = 512;
int32_t kp = 1011120;
int32_t ki = 1320*1000;
int32_t kd = 5280 * 1000;
uint8_t qn = 22;    // Set QN to 32 - DAC resolution
PID pidController = PID(setpoint, kp, ki, kd, qn);

const unsigned short DAC_OUTPUT_PIN = 2;
const unsigned short ADC_INPUT_PIN = 0;


void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(DAC_OUTPUT_PIN, OUTPUT);
  pinMode(ADC_INPUT_PIN, INPUT);

  pidController.setOutputMin(0);      // This is the default
  pidController.setOutputMax(1023);   // The Arduino has a 10-bit'analog' PWM output,
                                      // but the maximum output can be adjusted down.

  pidController.init(analogRead(ADC_INPUT_PIN));  // Initialize the pid controller to make sure there
                                      // are no output spikes

  // We will turn on the LED, once we are ready
  digitalWrite(13, HIGH);
}

void loop() {
  // The Teensy will lock up, if the input buffer overflows
  while (Serial.available()) {
   Serial.read();
  }

  static unsigned long lastTime = millis();    // Initialize lastTime *once* during the first loop iteration

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= pidPeriod) {    // Only run the pid controller
    unsigned short inputValue = analogRead(ADC_INPUT_PIN);  // Read a new analog value
    unsigned short outputValue = pidController.compute(inputValue);   // Compute the PID output
    analogWrite(DAC_OUTPUT_PIN, outputValue);    // Write it to the DAC

    lastTime = currentTime;
    Serial.println("------------------");
    Serial.print("Analog input: ");
    Serial.println(inputValue);
    Serial.print("DAC output: ");
    Serial.println(outputValue);
  }
}
```

## Performance Comparison
In terms of performance only two models of the [Teensy](https://www.pjrc.com/teensy/techspecs.html) platform were tested, because they have a wide range of 32-bit processors.

|Model|Processor|Clock Speed in MHz|Comments|
|--|--|--|--|
|Teensy LC|Cortex M0+|48|32-bit|
|Teensy 3.6|Cortex M4F|180|32-bit, FPU|

The full benchmark sketch can be found [here](extras/benchmark/benchmark.ino). It calculates the number of instruction cycles needed to run the `compute()` method.

For the floating point test, the code below was used, which was adapted from the [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library/). The `LIKELY`/`UNLIKELY` macro was used to tell the compiler about the default values and which conditional path was more likely. All benchmarks were compiled using the option Fastests with LTO (-O3 -flto):

The floating point code is simpler because overflow conditions only need to be checked at the end. It is therefore expected to run faster if there is a dedicated floating point unit available (that is single precision, 32-bit floats).

```c
#ifdef DOUBLE_PRECISION_MATH
  typedef double myFloatType;
#elif defined(SINGLE_PRECISION_MATH)
  typedef float myFloatType;
#endif

static inline const __attribute__((always_inline, unused)) myFloatType clamp(myFloatType value, myFloatType min, myFloatType max) {
  return (value < min) ? min : (value > max) ? max : value;
}

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
```
The spread in the number of Ops required to run one iteration of the `compute()` loop is the result of the branching required when saturating the output.

|Algorithm|Teensy LC|Teensy 3.6|
|--|--|--|
|Fixed Point Arithmetic|380-392 Ops/Call|88-104 Ops/Call|
|Single Precision Arithmetic|1522-1558 Ops/Call|48-55 Ops/Call|
|Double Precision Arithmetic|2827-2845 Ops/Call|1107-1200 Ops/Call|

In conclusion, it can be said, that unless you have an MCU with a dedicated FPU and do not need the precision of 32 bits, than it is recommended to use fixed point arithmetic for a 7-10x increase in performance.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/PatrickBaus/PID-CPP/tags). 

## Authors

* **Patrick Baus** - *Initial work* - [PatrickBaus](https://github.com/PatrickBaus)

## License

This project is licensed under the GPL v3 license - see the [LICENSE](LICENSE) file for details

## Acknowledgments

* [Brett Beauregard](https://github.com/br3ttb/) for his excellent work in explaining all the PID details and improvements
