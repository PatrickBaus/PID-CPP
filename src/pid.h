#ifndef PID_H
#define PID_H

#include <stdint.h>

#define LIKELY(x)       __builtin_expect(!!(x), 1)
#define UNLIKELY(x)     __builtin_expect(!!(x), 0)

static inline const int32_t clamp(const int32_t value, const int32_t min, const int32_t max) __attribute__((always_inline, unused));
static inline const int32_t clamp(const int32_t value, const int32_t min, const int32_t max) {
    return (value < min) ? min : (value > max) ? max : value;
}
/* The instruction sets for different ARM processors can be found here
 * https://en.wikipedia.org/wiki/ARM_Cortex-M#Instruction_sets
 */
static const inline int32_t signed_add_saturated_32_and_32(const int32_t a, const int32_t b) __attribute__((always_inline, unused));
static const inline int32_t signed_add_saturated_32_and_32(const int32_t a, const int32_t b)
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

static const inline int32_t signed_subtract_saturated_32_and_32(const int32_t a, const int32_t b) __attribute__((always_inline, unused));
static const inline int32_t signed_subtract_saturated_32_and_32(const int32_t a, const int32_t b)
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

static const inline int32_t signed_multiply_accumulate_saturated_32QN_and_32QN(const int32_t acc, const int32_t a, const int32_t b, const uint8_t qn) __attribute__((always_inline, unused));
static const inline int32_t signed_multiply_accumulate_saturated_32QN_and_32QN(const int32_t acc, const int32_t a, const int32_t b, const uint8_t qn) {
    // Don't worry about the 64-bit ints, it is one op using the DSP extensions (smlal instruction)
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

/* To get the Assembly version of this function go to https://godbolt.org/
 * Using the ARM gcc 5.4.1 (none) compiler with the follwing options:
 * "-O3 -Wl,  -mcpu=cortex-m4 -lstdc++ -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -larm_cortexM4lf_math"
 * This resembles the Arduino options "fastest".
 * To disassemble the ELF code to compare it to the code procuced by "Compiler Explorer"  do the following:
 * - Add __attribute__((noinline)), to make sure the compiler does not inline the code
 * - Compile it using the option "fastest" or even better "fastest + LTO"
 * - Then go to /tmp/arduino_build_XXX , where XXX is the number found in the Arduino console (requires verbose output)
 * - Finally run ~/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-objdump -C -d your_sketch.ino.elf (replacing "your_sketch" with the name of your sketch)
 */
static const inline int32_t signed_multiply_accumulate_saturated_32_and_32QN(const int32_t acc, const int32_t a, const int32_t b) __attribute__((always_inline, unused));
static const inline int32_t signed_multiply_accumulate_saturated_32_and_32QN(const int32_t acc, const int32_t a, const int32_t b) {
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

enum FeedbackDirection {
    feedbackPositive = 0,
    feedbackNegative = 1,
};

enum ProportionalGain {
    proportionalToInput = 0,
    proportionalToError = 1,
};

class PID {
    public:
        PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, uint8_t _qn, FeedbackDirection feedbackDirection, ProportionalGain proportionalGain);
        PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, uint8_t _qn, FeedbackDirection feedbackDirection);
        PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, uint8_t _qn);

        const uint32_t compute(uint32_t input);

        void setTunings(const int32_t kp, const int32_t ki, const int32_t kd);
        void setTunings(const int32_t kp, const int32_t ki, const int32_t kd, const ProportionalGain proportionalGain);
        const uint32_t getKp();
        const uint32_t getKi();
        const uint32_t getKd();
        void setSetpoint(const uint32_t value);
        const uint32_t getSetpoint();
        void setControllerFeedback(const FeedbackDirection feedbackDirection);
        void setOutputMin(const uint32_t value);
        void setOutputMax(const uint32_t value);
        void updateOutput(const uint32_t value);
        void init(const uint32_t initialInput);
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
#endif

