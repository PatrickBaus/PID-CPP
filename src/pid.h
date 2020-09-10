#ifndef PID_H
#define PID_H

#include <stdint.h>    // uint8_t, etc.

#define LIKELY(x)       __builtin_expect(!!(x), 1)
#define UNLIKELY(x)     __builtin_expect(!!(x), 0)

namespace Pid {
  static inline int32_t clamp(const int32_t value, const int32_t min, const int32_t max) __attribute__((always_inline, unused));
  static inline int32_t clamp(const int32_t value, const int32_t min, const int32_t max) {
      return (value < min) ? min : (value > max) ? max : value;
  }
  /* The following two functions contains special assembly optimizations for the Cortex M4. These optimizations require
   * the QADD and QSUB processor instrutions found on the Cortex M4 (and more advanced architectures).
   * The instruction sets for different ARM processors can be found here (look for 'Saturated instructions')
   * The function signed_add_saturated_32_and_32() is not used in the PID library and only included for educational
   * purposes.
   * https://en.wikipedia.org/wiki/ARM_Cortex-M#Instruction_sets
   */
  static inline int32_t signed_add_saturated_32_and_32(const int32_t a, const int32_t b) __attribute__((always_inline, unused));
  static inline int32_t signed_add_saturated_32_and_32(const int32_t a, const int32_t b)
  {
      #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)    // Teensy 3.x (Cortex M4)
      uint32_t result;
      asm volatile("qadd %0, %1, %2" : "=r" (result) : "r" (a), "r" (b));
      return result;
      #else
      uint32_t ua = (uint32_t)a;
      uint32_t result = ua + (uint32_t)b;
      ua = (ua >> 31) + INT32_MAX;

      if ((int32_t)((ua ^ b) | ~(b ^ result)) >= 0) {
          result = ua;
      }
      return result;
      #endif
  }

  static inline int32_t signed_subtract_saturated_32_and_32(const int32_t a, const int32_t b) __attribute__((always_inline, unused));
  static inline int32_t signed_subtract_saturated_32_and_32(const int32_t a, const int32_t b)
  {
      #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)    // Teensy 3.x (Cortex M4)
      int32_t result;
      asm volatile("qsub %0, %1, %2" : "=r" (result) : "r" (a), "r" (b));
      return result;
      #else
      uint32_t ua = (uint32_t)a;
      uint32_t result = ua - (uint32_t)b;
      ua = (ua >> 31) + INT32_MAX;

      if ((int32_t)((ua ^ b) & (ua ^ result)) < 0) {
          result = ua;
      }
      return result;
      #endif
  }

  /* To get the assembly version of these functions go to https://godbolt.org/
   * Use the ARM gcc 5.4.1 (none) compiler with the follwing options:
   * "-O3 -Wl,  -mcpu=cortex-m4 -lstdc++ -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -larm_cortexM4lf_math"
   * This resembles the Arduino options "fastest".
   * To disassemble the ELF code to compare it to the code procuced by "Compiler Explorer" do the following:
   * - Add __attribute__((noinline)), to make sure the compiler does not inline the code
   * - Compile it using the option "fastest" or even better "fastest + LTO"
   * - Then go to /tmp/arduino_build_XXX , where XXX is the number found in the Arduino console (requires verbose output)
   * - Finally run ~/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-objdump -C -d YOUR_SKETCH.ino.elf (replacing "YOUR_SKETCH" with the name of your sketch)
   */
  static inline int32_t signed_multiply_accumulate_saturated_32QN_and_32QN(const int32_t acc, const int32_t a, const int32_t b, const uint8_t qn) __attribute__((always_inline, unused));
  static inline int32_t signed_multiply_accumulate_saturated_32QN_and_32QN(const int32_t acc, const int32_t a, const int32_t b, const uint8_t qn) {
      // Don't worry about the 64-bit integers, it is one op using the DSP extensions (SMLAL instruction)
      // The compiler switch -O2 or -O3 is required! (Option Faster/Fastest)
      int64_t result = ((int64_t)acc << qn) + (int64_t) a * (int64_t) b;
      result >>= qn;    // When mulitplying two Qm.n numbers you will end up with a Qm^2.n^2 number, but we need Qm^2.n (fixed precision)
      int32_t hi = (int32_t)(result >> 32);
      int32_t lo = (int32_t)result;
      // We will branch here, because almost always the branch will *not* be hit, because we will not
      // saturate. Unfortunately the Cortex Mx does not support CMOVs, so we cannot do this non-branching.
      if (UNLIKELY(hi != (lo >> 31))) {
          return ((uint32_t) (a ^ b) >> 31) + INT32_MAX;
      }
      return result;
  }

  static inline int32_t signed_multiply_accumulate_saturated_32_and_32QN(const int32_t acc, const int32_t a, const int32_t b) __attribute__((always_inline, unused));
  static inline int32_t signed_multiply_accumulate_saturated_32_and_32QN(const int32_t acc, const int32_t a, const int32_t b) {
      // Don't worry about the 64-bit int, it is one op using the DSP extensions (smlal instruction)
      // The compiler switch -O2 or -O3 is required! (Option Faster/Fastest)
      int64_t result = acc + (int64_t) a * (int64_t) b;
      int32_t hi = (int32_t)(result >> 32);
      int32_t lo = (int32_t)result;
      // We will branch here, because almost always the branch will *not* be hit, because we will not
      // saturate. Unfortunately the Cortex Mx does not support CMOVs, so we cannot do this non-branching.
      if (UNLIKELY(hi != (lo >> 31))) {
          return ((uint32_t) (a ^ b) >> 31) + INT32_MAX;
      }
      return result;
  }

  enum FeedbackDirection : bool {
      feedbackNegative = 0,
      feedbackPositive = 1,
  };

  enum ProportionalGain : bool {
      proportionalToInput = 0,
      proportionalToError = 1,
  };

  class PID {
      public:
          PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn, const FeedbackDirection feedbackDirection, const ProportionalGain proportionalGain);
          PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn, const FeedbackDirection feedbackDirection);
          PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn);

          uint32_t compute(const uint32_t input);

          void setTunings(const int32_t kp, const int32_t ki, const int32_t kd);
          void setTunings(const int32_t kp, const int32_t ki, const int32_t kd, const ProportionalGain proportionalGain);
          uint32_t getKp();
          uint32_t getKi();
          uint32_t getKd();
          void setSetpoint(const uint32_t value);
          uint32_t getSetpoint();
          int32_t getIntegratorError();
          void setControllerFeedback(const FeedbackDirection feedbackDirection);
          void setOutputMin(const uint32_t value);
          void setOutputMax(const uint32_t value);
          void updateOutput(const uint32_t value);
          void init(const uint32_t initialInput);
          void init(const uint32_t initialInput, const int32_t initialErrorSum);
      private:
          int32_t kp, ki, kd;
          FeedbackDirection feedbackDirection;
          ProportionalGain proportionalGain;
          uint8_t qn;
          int32_t outputMin = INT32_MIN;
          int32_t outputMax = INT32_MAX;
          int32_t previousInput = 0;
          int32_t errorSum = 0;
      protected:
          int32_t setpoint;
  };
}   // namespace pid
#endif

