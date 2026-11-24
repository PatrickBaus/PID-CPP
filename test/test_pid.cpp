/*
 *  Host based unit tests for the PID library.
 *
 *  The fixed point controller is checked against two independent models:
 *   - ReferencePid: an exact integer model using wide (64 bit) arithmetic with explicit saturation. The library must
 *     match it bit for bit.
 *   - a textbook double precision PID in real units. The library must match it to within one output LSB, which is
 *     the quantisation caused by truncating the fractional bits of the output.
 */
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "pid.h"

using namespace Pid;

namespace {
  const int64_t I32_MIN = INT32_MIN;
  const int64_t I32_MAX = INT32_MAX;
  const uint32_t MIDSCALE = 0x80000000u;

  int64_t saturate(const int64_t value) {
      return value < I32_MIN ? I32_MIN : (value > I32_MAX ? I32_MAX : value);
  }

  int64_t limit(const int64_t value, const int64_t min, const int64_t max) {
      return value < min ? min : (value > max ? max : value);
  }

  // The values every 32 bit operand is tested with. They include all the corner cases of the saturating maths.
  const std::vector<int32_t>& interestingValues() {
      static const std::vector<int32_t> values = {
          INT32_MIN, INT32_MIN + 1, INT32_MIN + 2, -0x40000001, -0x40000000, -0x3FFFFFFF, -65537, -65536, -65535,
          -46341, -46340, -3, -2, -1, 0, 1, 2, 3, 46340, 46341, 65535, 65536, 65537, 0x3FFFFFFF, 0x40000000,
          0x40000001, INT32_MAX - 2, INT32_MAX - 1, INT32_MAX,
      };
      return values;
  }

  /*
   * The helpers in pid.h are always inlined, which creates a new copy at every call site. The tests call them through
   * these wrappers, so that there is exactly one copy and its coverage is the coverage of the tests.
   */
  __attribute__((noinline)) int32_t clampValue(const int32_t value, const int32_t min, const int32_t max) {
      return clamp(value, min, max);
  }

  __attribute__((noinline)) int32_t addSaturated(const int32_t a, const int32_t b) {
      return signed_add_saturated_32_and_32(a, b);
  }

  __attribute__((noinline)) int32_t subtractSaturated(const int32_t a, const int32_t b) {
      return signed_subtract_saturated_32_and_32(a, b);
  }

  __attribute__((noinline)) int32_t multiplyAccumulate(const int32_t acc, const int32_t a, const int32_t b) {
      return signed_multiply_accumulate_saturated_32_and_32QN(acc, a, b);
  }

  __attribute__((noinline)) int32_t multiplyAccumulateQn(const int32_t acc, const int32_t a, const int32_t b, const uint8_t qn) {
      return signed_multiply_accumulate_saturated_32QN_and_32QN(acc, a, b, qn);
  }

  /*
   * Exact model of the controller. All values are kept in 64 bit integers, so there is no need for any tricks to
   * detect an overflow. The inputs, the setpoint and the output are unsigned numbers (offset binary), the gains are
   * signed Qm.n fixed point numbers and the integrator is a signed Qm.n number relative to the midscale output.
   */
  struct ReferencePid {
      int64_t setpoint;
      int64_t kp, ki, kd;    // Signed, i.e. negative for a controller with negative feedback
      uint8_t qn;
      bool proportionalOnError;
      int64_t outputMin;
      int64_t outputMax;
      int64_t previousInput;
      int64_t errorSum;

      ReferencePid(const uint32_t _setpoint, const int64_t _kp, const int64_t _ki, const int64_t _kd, const uint8_t _qn, const bool _proportionalOnError = true)
          : setpoint(_setpoint), kp(_kp), ki(_ki), kd(_kd), qn(_qn), proportionalOnError(_proportionalOnError),
          outputMin(I32_MIN), outputMax(I32_MAX), previousInput(0), errorSum(0) {}

      static int64_t fromOutput(const uint32_t value, const uint8_t qn) {
          return ((int64_t)value << qn) - ((int64_t)1 << 31);
      }

      void setOutputMin(const uint32_t value) { this->outputMin = fromOutput(value, this->qn); }
      void setOutputMax(const uint32_t value) { this->outputMax = fromOutput(value, this->qn); }

      uint32_t compute(const uint32_t input) {
          const int64_t error = saturate(this->setpoint - (int64_t)input);
          const int64_t dInputNegative = saturate(this->previousInput - (int64_t)input);
          this->previousInput = input;

          this->errorSum = saturate(this->errorSum + this->ki * error);
          if (!this->proportionalOnError) {
              this->errorSum = saturate(this->errorSum + this->kp * dInputNegative);
          }
          this->errorSum = limit(this->errorSum, this->outputMin, this->outputMax);

          int64_t output = saturate(this->errorSum + this->kd * dInputNegative);
          if (this->proportionalOnError) {
              output = saturate(output + this->kp * error);
          }
          output = limit(output, this->outputMin, this->outputMax);
          return (uint32_t)((output + ((int64_t)1 << 31)) >> this->qn);
      }
  };
}   // namespace

TEST_SUITE("saturating maths") {
    TEST_CASE("clamp() limits a value to the range [min, max]") {
        CHECK(clampValue(5, -10, 10) == 5);
        CHECK(clampValue(-10, -10, 10) == -10);
        CHECK(clampValue(10, -10, 10) == 10);
        CHECK(clampValue(-11, -10, 10) == -10);
        CHECK(clampValue(11, -10, 10) == 10);
        CHECK(clampValue(INT32_MIN, -10, 10) == -10);
        CHECK(clampValue(INT32_MAX, -10, 10) == 10);
        CHECK(clampValue(0, INT32_MIN, INT32_MAX) == 0);
        CHECK(clampValue(INT32_MIN, INT32_MIN, INT32_MAX) == INT32_MIN);
        CHECK(clampValue(INT32_MAX, INT32_MIN, INT32_MAX) == INT32_MAX);
    }

    TEST_CASE("signed_add_saturated_32_and_32() matches a 64 bit reference") {
        for (const int32_t a : interestingValues()) {
            for (const int32_t b : interestingValues()) {
                CAPTURE(a);
                CAPTURE(b);
                CHECK(addSaturated(a, b) == saturate((int64_t)a + b));
            }
        }
        // Spot checks using literals, so that an error in the reference does not go unnoticed
        CHECK(addSaturated(1, 2) == 3);
        CHECK(addSaturated(INT32_MAX, 1) == INT32_MAX);
        CHECK(addSaturated(INT32_MIN, -1) == INT32_MIN);
        CHECK(addSaturated(INT32_MIN, INT32_MAX) == -1);
    }

    TEST_CASE("signed_subtract_saturated_32_and_32() matches a 64 bit reference") {
        for (const int32_t a : interestingValues()) {
            for (const int32_t b : interestingValues()) {
                CAPTURE(a);
                CAPTURE(b);
                CHECK(subtractSaturated(a, b) == saturate((int64_t)a - b));
            }
        }
        CHECK(subtractSaturated(1, 2) == -1);
        CHECK(subtractSaturated(INT32_MAX, -1) == INT32_MAX);
        CHECK(subtractSaturated(INT32_MIN, 1) == INT32_MIN);
        CHECK(subtractSaturated(0, INT32_MIN) == INT32_MAX);
        CHECK(subtractSaturated(-1, INT32_MIN) == INT32_MAX);
    }

    TEST_CASE("the saturating add and subtract match a 64 bit reference for random operands") {
        std::mt19937 generator(0x5eed);
        std::uniform_int_distribution<int32_t> distribution(INT32_MIN, INT32_MAX);
        for (int i = 0; i < 100000; i++) {
            const int32_t a = distribution(generator);
            const int32_t b = distribution(generator);
            CAPTURE(a);
            CAPTURE(b);
            REQUIRE(addSaturated(a, b) == saturate((int64_t)a + b));
            REQUIRE(subtractSaturated(a, b) == saturate((int64_t)a - b));
        }
    }

    TEST_CASE("signed_multiply_accumulate_saturated_32_and_32QN() matches a 128 bit reference") {
        for (const int32_t acc : interestingValues()) {
            for (const int32_t a : interestingValues()) {
                for (const int32_t b : interestingValues()) {
                    CAPTURE(acc);
                    CAPTURE(a);
                    CAPTURE(b);
                    const __int128 expected = (__int128)acc + (__int128)a * b;
                    const int32_t reference = expected < I32_MIN ? INT32_MIN : (expected > I32_MAX ? INT32_MAX : (int32_t)expected);
                    CHECK(multiplyAccumulate(acc, a, b) == reference);
                }
            }
        }
        CHECK(multiplyAccumulate(10, 3, 4) == 22);
        CHECK(multiplyAccumulate(10, -3, 4) == -2);
        CHECK(multiplyAccumulate(INT32_MAX, 1, 1) == INT32_MAX);
        CHECK(multiplyAccumulate(INT32_MIN, -1, 1) == INT32_MIN);
        CHECK(multiplyAccumulate(0, INT32_MIN, INT32_MIN) == INT32_MAX);
        CHECK(multiplyAccumulate(0, INT32_MIN, INT32_MAX) == INT32_MIN);
        // The accumulator has the opposite sign of the product and almost cancels it
        CHECK(multiplyAccumulate(INT32_MIN, 65536, 32768) == 0);
        CHECK(multiplyAccumulate(INT32_MAX, -65536, 32768) == -1);
    }

    TEST_CASE("signed_multiply_accumulate_saturated_32QN_and_32QN() matches a 128 bit reference") {
        for (uint8_t qn = 0; qn < 32; qn++) {
            for (const int32_t acc : interestingValues()) {
                for (const int32_t a : interestingValues()) {
                    for (const int32_t b : interestingValues()) {
                        CAPTURE((int)qn);
                        CAPTURE(acc);
                        CAPTURE(a);
                        CAPTURE(b);
                        // acc + a * b / 2^qn, rounded towards negative infinity
                        __int128 expected = (__int128)acc * ((__int128)1 << qn) + (__int128)a * b;
                        expected = (expected - (((expected % ((__int128)1 << qn)) + ((__int128)1 << qn)) % ((__int128)1 << qn))) / ((__int128)1 << qn);
                        const int32_t reference = expected < I32_MIN ? INT32_MIN : (expected > I32_MAX ? INT32_MAX : (int32_t)expected);
                        REQUIRE(multiplyAccumulateQn(acc, a, b, qn) == reference);
                    }
                }
            }
        }
        // 1.5 + 2.5 * 3.0 = 9.0 in Q16.16
        CHECK(multiplyAccumulateQn(0x18000, 0x28000, 0x30000, 16) == 0x90000);
        // -1.5 + 2.5 * 3.0 = 6.0 in Q16.16. A negative accumulator must not invoke undefined behaviour.
        CHECK(multiplyAccumulateQn(-0x18000, 0x28000, 0x30000, 16) == 0x60000);
        // -1.5 - 2.5 * 3.0 = -9.0 in Q16.16
        CHECK(multiplyAccumulateQn(-0x18000, -0x28000, 0x30000, 16) == -0x90000);
        // 256.0 * 256.0 does not fit into Q16.16
        CHECK(multiplyAccumulateQn(0, 0x1000000, 0x1000000, 16) == INT32_MAX);
        CHECK(multiplyAccumulateQn(0, -0x1000000, 0x1000000, 16) == INT32_MIN);
    }
}

TEST_SUITE("configuration") {
    TEST_CASE("the constructor stores the setpoint") {
        PID pid(12345, 1, 2, 3, 16);
        CHECK(pid.getSetpoint() == 12345);
        CHECK(pid.getIntegratorError() == 0);
    }

    TEST_CASE("the setpoint covers the full unsigned 32 bit range") {
        PID pid(0, 1, 2, 3, 16);
        CHECK(pid.getSetpoint() == 0);
        pid.setSetpoint(0x80000000u);
        CHECK(pid.getSetpoint() == 0x80000000u);
        pid.setSetpoint(UINT32_MAX);
        CHECK(pid.getSetpoint() == UINT32_MAX);
    }

    TEST_CASE("the getters return the magnitude of the gains for either feedback direction") {
        SUBCASE("default (negative) feedback") {
            PID pid(0, 100, 200, 300, 16);
            CHECK(pid.getKp() == 100);
            CHECK(pid.getKi() == 200);
            CHECK(pid.getKd() == 300);
        }
        SUBCASE("negative feedback") {
            PID pid(0, 100, 200, 300, 16, feedbackNegative);
            CHECK(pid.getKp() == 100);
            CHECK(pid.getKi() == 200);
            CHECK(pid.getKd() == 300);
        }
        SUBCASE("positive feedback") {
            PID pid(0, 100, 200, 300, 16, feedbackPositive);
            CHECK(pid.getKp() == 100);
            CHECK(pid.getKi() == 200);
            CHECK(pid.getKd() == 300);
        }
        SUBCASE("the sign of the gains is ignored") {
            PID negative(0, -100, -200, -300, 16, feedbackNegative);
            CHECK(negative.getKp() == 100);
            CHECK(negative.getKi() == 200);
            CHECK(negative.getKd() == 300);
            PID positive(0, -100, -200, -300, 16, feedbackPositive);
            CHECK(positive.getKp() == 100);
            CHECK(positive.getKi() == 200);
            CHECK(positive.getKd() == 300);
        }
        SUBCASE("the gains are not truncated to 16 bit") {
            PID pid(0, 1011120, 1320000, 5280000, 22);
            CHECK(pid.getKp() == 1011120);
            CHECK(pid.getKi() == 1320000);
            CHECK(pid.getKd() == 5280000);
        }
        SUBCASE("zero gains") {
            PID pid(0, 0, 0, 0, 16, feedbackPositive);
            CHECK(pid.getKp() == 0);
            CHECK(pid.getKi() == 0);
            CHECK(pid.getKd() == 0);
        }
    }

    TEST_CASE("setTunings() replaces the gains") {
        PID pid(0, 1, 2, 3, 16);
        pid.setTunings(40, -50, 60);
        CHECK(pid.getKp() == 40);
        CHECK(pid.getKi() == 50);
        CHECK(pid.getKd() == 60);
        pid.setTunings(-7, 8, -9, proportionalToInput);
        CHECK(pid.getKp() == 7);
        CHECK(pid.getKi() == 8);
        CHECK(pid.getKd() == 9);
    }

    TEST_CASE("the extreme gains INT32_MIN and INT32_MAX are handled without overflow") {
        // The magnitude of INT32_MIN cannot be represented as a positive gain, the closest value is INT32_MAX.
        SUBCASE("negative feedback") {
            PID pid(100, INT32_MIN, INT32_MAX, INT32_MIN, 0, feedbackNegative);
            CHECK(pid.getKp() == 0x80000000u);
            CHECK(pid.getKi() == 0x7FFFFFFFu);
            CHECK(pid.getKd() == 0x80000000u);
            // error = 1, ki * error = -(2^31 - 1), kp * error = -2^31 -> saturates at the bottom
            pid.init(99);
            CHECK(pid.compute(99) == 0);
            CHECK(pid.getIntegratorError() == INT32_MIN + 1);
        }
        SUBCASE("positive feedback") {
            PID pid(100, INT32_MIN, INT32_MAX, INT32_MIN, 0, feedbackPositive);
            CHECK(pid.getKp() == 0x7FFFFFFFu);
            CHECK(pid.getKi() == 0x7FFFFFFFu);
            CHECK(pid.getKd() == 0x7FFFFFFFu);
            // error = -1, ki * error = -(2^31 - 1), the integrator is at INT32_MIN + 1
            pid.init(101);
            CHECK(pid.compute(101) == 0);
            CHECK(pid.getIntegratorError() == INT32_MIN + 1);
        }
        SUBCASE("changing the direction back and forth") {
            PID pid(100, INT32_MIN, INT32_MIN, INT32_MIN, 0, feedbackNegative);
            pid.setControllerFeedback(feedbackPositive);
            CHECK(pid.getKp() == 0x7FFFFFFFu);
            pid.setControllerFeedback(feedbackNegative);
            CHECK(pid.getKp() == 0x7FFFFFFFu);
        }
    }
}

TEST_SUITE("controller output") {
    TEST_CASE("a controller without gains outputs midscale") {
        for (uint8_t qn = 0; qn < 32; qn++) {
            CAPTURE((int)qn);
            PID pid(1000, 0, 0, 0, qn);
            CHECK(pid.compute(0) == (MIDSCALE >> qn));
            CHECK(pid.compute(5000) == (MIDSCALE >> qn));
            CHECK(pid.getIntegratorError() == 0);
        }
    }

    TEST_CASE("proportional term") {
        // kp = 2.5 in Q20.12. The output is a 20 bit number with the midscale value 2^19.
        const uint8_t qn = 12;
        const int32_t kp = 5 << (qn - 1);
        const uint32_t midscale = 1u << 19;

        SUBCASE("negative feedback: a positive error lowers the output") {
            PID pid(1000, kp, 0, 0, qn, feedbackNegative);
            pid.init(1000);
            CHECK(pid.compute(1000) == midscale);
            CHECK(pid.compute(900) == midscale - 250);    // error = +100
            CHECK(pid.compute(1100) == midscale + 250);   // error = -100
            CHECK(pid.compute(1000) == midscale);         // There is no memory in the proportional term
        }
        SUBCASE("the default is negative feedback") {
            PID pid(1000, kp, 0, 0, qn);
            pid.init(1000);
            CHECK(pid.compute(900) == midscale - 250);
        }
        SUBCASE("positive feedback: a positive error raises the output") {
            PID pid(1000, kp, 0, 0, qn, feedbackPositive);
            pid.init(1000);
            CHECK(pid.compute(900) == midscale + 250);
            CHECK(pid.compute(1100) == midscale - 250);
        }
        SUBCASE("the fractional bits of the output are truncated") {
            PID pid(1000, kp, 0, 0, qn, feedbackPositive);
            CHECK(pid.compute(999) == midscale + 2);    // 2.5 -> 2
            CHECK(pid.compute(1001) == midscale - 3);   // -2.5 -> -3 (rounding towards negative infinity)
        }
    }

    TEST_CASE("integral term") {
        const uint8_t qn = 12;
        const int32_t ki = 1 << (qn - 2);   // 0.25
        const uint32_t midscale = 1u << 19;

        PID pid(1000, 0, ki, 0, qn, feedbackPositive);
        pid.init(1000);
        for (uint32_t i = 1; i <= 8; i++) {
            CAPTURE(i);
            CHECK(pid.compute(990) == midscale + (10 * i) / 4);   // 2.5 per iteration
            CHECK(pid.getIntegratorError() == (int32_t)(10 * i * ki));
        }
        // The integrator holds its value if there is no error ...
        CHECK(pid.compute(1000) == midscale + 20);
        CHECK(pid.compute(1000) == midscale + 20);
        // ... and winds down if the error changes its sign
        CHECK(pid.compute(1040) == midscale + 10);
        CHECK(pid.compute(1040) == midscale);
        CHECK(pid.compute(1040) == midscale - 10);
        CHECK(pid.getIntegratorError() == -40 * ki);
    }

    TEST_CASE("derivative term") {
        const uint8_t qn = 12;
        const int32_t kd = 3 << qn;   // 3.0
        const uint32_t midscale = 1u << 19;

        SUBCASE("it acts on the change of the input") {
            PID pid(1000, 0, 0, kd, qn, feedbackPositive);
            pid.init(1000);
            CHECK(pid.compute(1000) == midscale);
            CHECK(pid.compute(1010) == midscale - 30);    // A rising input lowers the output
            CHECK(pid.compute(1010) == midscale);
            CHECK(pid.compute(1005) == midscale + 15);
            CHECK(pid.getIntegratorError() == 0);
        }
        SUBCASE("negative feedback inverts it") {
            PID pid(1000, 0, 0, kd, qn, feedbackNegative);
            pid.init(1000);
            CHECK(pid.compute(1010) == midscale + 30);
        }
        SUBCASE("a setpoint change does not cause a spike") {
            PID pid(1000, 0, 0, kd, qn, feedbackPositive);
            pid.init(1000);
            pid.setSetpoint(2000);
            CHECK(pid.compute(1000) == midscale);
        }
        SUBCASE("init() sets the previous input to prevent a spike in the first iteration") {
            PID initialized(1000, 0, 0, kd, qn, feedbackPositive);
            initialized.init(1000);
            CHECK(initialized.compute(1000) == midscale);
            // Without init() the previous input is 0
            PID uninitialized(1000, 0, 0, kd, qn, feedbackPositive);
            CHECK(uninitialized.compute(1000) == midscale - 3000);
            CHECK(uninitialized.compute(1000) == midscale);
        }
    }

    TEST_CASE("proportional on input") {
        const uint8_t qn = 12;
        const int32_t kp = 2 << qn;         // 2.0
        const int32_t ki = 1 << (qn - 1);   // 0.5
        const uint32_t midscale = 1u << 19;

        SUBCASE("a setpoint change does not cause a step of the output") {
            PID onInput(1000, kp, ki, 0, qn, feedbackPositive, proportionalToInput);
            onInput.init(1000);
            onInput.setSetpoint(1100);
            CHECK(onInput.compute(1000) == midscale + 50);    // Only the integral term: 0.5 * 100

            PID onError(1000, kp, ki, 0, qn, feedbackPositive, proportionalToError);
            onError.init(1000);
            onError.setSetpoint(1100);
            CHECK(onError.compute(1000) == midscale + 50 + 200);
        }
        SUBCASE("the proportional term is accumulated in the integrator") {
            PID pid(1000, kp, 0, 0, qn, feedbackPositive, proportionalToInput);
            pid.init(1000);
            CHECK(pid.compute(1010) == midscale - 20);
            CHECK(pid.getIntegratorError() == -10 * kp);
            CHECK(pid.compute(1010) == midscale - 20);
            CHECK(pid.compute(990) == midscale + 20);
            CHECK(pid.getIntegratorError() == 10 * kp);
        }
        SUBCASE("setTunings() with three arguments keeps the mode") {
            PID pid(1000, kp, 0, 0, qn, feedbackPositive, proportionalToInput);
            pid.init(1000);
            pid.setTunings(4 << qn, 0, 0);
            CHECK(pid.compute(1010) == midscale - 40);
            CHECK(pid.compute(1010) == midscale - 40);    // Still proportional to the input
        }
        SUBCASE("setTunings() with four arguments changes the mode") {
            PID pid(1000, kp, 0, 0, qn, feedbackPositive, proportionalToError);
            pid.init(1000);
            CHECK(pid.compute(1010) == midscale - 20);
            CHECK(pid.getIntegratorError() == 0);
            pid.setTunings(kp, 0, 0, proportionalToInput);
            CHECK(pid.compute(1010) == midscale);         // The input did not change
            CHECK(pid.compute(1020) == midscale - 20);
            CHECK(pid.getIntegratorError() == -10 * kp);
            pid.setTunings(kp, 0, 0, proportionalToError);
            CHECK(pid.compute(1020) == midscale - 20 - 40);
        }
    }

    TEST_CASE("the gains can be changed at runtime") {
        const uint8_t qn = 12;
        const uint32_t midscale = 1u << 19;
        PID pid(1000, 1 << qn, 1 << qn, 0, qn, feedbackPositive);
        pid.init(1000);
        CHECK(pid.compute(990) == midscale + 10 + 10);
        // The integrator is kept, the new gains only affect the following iterations
        pid.setTunings(3 << qn, 2 << qn, 0);
        CHECK(pid.getIntegratorError() == 10 << qn);
        CHECK(pid.compute(990) == midscale + (10 + 20) + 30);
    }

    TEST_CASE("setControllerFeedback() inverts the response of all terms") {
        const uint8_t qn = 12;
        const uint32_t midscale = 1u << 19;
        PID pid(1000, 1 << qn, 2 << qn, 4 << qn, qn, feedbackPositive);
        pid.init(1000);
        // error = 10, dInput = -10: P = 10, I = 20, D = 40
        CHECK(pid.compute(990) == midscale + 70);

        pid.setControllerFeedback(feedbackNegative);
        CHECK(pid.getKp() == (1u << qn));
        CHECK(pid.getKi() == (2u << qn));
        CHECK(pid.getKd() == (4u << qn));
        pid.init(1000);
        CHECK(pid.compute(990) == midscale - 70);

        pid.setControllerFeedback(feedbackPositive);
        pid.init(1000);
        CHECK(pid.compute(990) == midscale + 70);
    }
}

TEST_SUITE("output limits and anti-windup") {
    const uint8_t qn = 20;    // 12 bit output
    const uint32_t midscale = 1u << 11;

    TEST_CASE("without limits the output saturates at the limits of the output word") {
        PID pid(1000, 100 << qn, 0, 0, qn, feedbackPositive);
        pid.init(1000);
        CHECK(pid.compute(900) == 4095);
        CHECK(pid.compute(1100) == 0);
    }

    TEST_CASE("the output is limited to [outputMin, outputMax]") {
        PID pid(1000, 1 << qn, 0, 0, qn, feedbackPositive);
        pid.setOutputMin(2000);
        pid.setOutputMax(2100);
        pid.init(1000);
        CHECK(pid.compute(1000) == midscale);
        CHECK(pid.compute(1000 - 52) == 2100);
        CHECK(pid.compute(1000 - 53) == 2100);
        CHECK(pid.compute(0) == 2100);
        CHECK(pid.compute(1000 + 48) == 2000);
        CHECK(pid.compute(1000 + 49) == 2000);
        CHECK(pid.compute(UINT32_MAX) == 2000);
    }

    TEST_CASE("the limits cover the full output range") {
        PID pid(1000, 100 << qn, 0, 0, qn, feedbackPositive);
        pid.setOutputMin(0);
        pid.setOutputMax(4095);
        pid.init(1000);
        CHECK(pid.compute(900) == 4095);
        CHECK(pid.compute(1100) == 0);
    }

    TEST_CASE("the integrator does not wind up while the output is saturated") {
        const int32_t ki = 1 << qn;   // 1.0
        PID pid(1000, 0, ki, 0, qn, feedbackPositive);
        pid.setOutputMin(1000);
        pid.setOutputMax(3000);
        pid.init(1000);

        // A large error for a long time drives the output into the upper limit ...
        uint32_t output = 0;
        for (int i = 0; i < 1000; i++) {
            output = pid.compute(900);
        }
        CHECK(output == 3000);
        CHECK(pid.getIntegratorError() == (int32_t)((3000 - midscale) << qn));
        // ... and it leaves the limit immediately when the sign of the error changes.
        CHECK(pid.compute(1001) == 2999);
        CHECK(pid.compute(1010) == 2989);

        // The same applies to the lower limit
        for (int i = 0; i < 1000; i++) {
            output = pid.compute(1100);
        }
        CHECK(output == 1000);
        CHECK(pid.getIntegratorError() == -(int32_t)((midscale - 1000) << qn));
        CHECK(pid.compute(999) == 1001);
        CHECK(pid.compute(990) == 1011);
    }

    TEST_CASE("the integrator saturates at the limits of the 32 bit accumulator") {
        PID pid(0x80000000u, 0, INT32_MAX, 0, 0, feedbackPositive);
        pid.init(0);
        CHECK(pid.compute(0) == UINT32_MAX);
        CHECK(pid.getIntegratorError() == INT32_MAX);
        CHECK(pid.compute(0) == UINT32_MAX);
        CHECK(pid.compute(UINT32_MAX) == 0);
        CHECK(pid.getIntegratorError() == INT32_MIN);
        CHECK(pid.compute(UINT32_MAX) == 0);
    }

    TEST_CASE("the proportional on input term does not wind up either") {
        PID pid(1000, 1 << qn, 0, 0, qn, feedbackPositive, proportionalToInput);
        pid.setOutputMin(2000);
        pid.setOutputMax(2100);
        pid.init(1000);
        CHECK(pid.compute(500) == 2100);
        CHECK(pid.getIntegratorError() == (int32_t)((2100 - midscale) << qn));
        CHECK(pid.compute(510) == 2090);
        CHECK(pid.compute(5000) == 2000);
        CHECK(pid.getIntegratorError() == -(int32_t)((midscale - 2000) << qn));
        CHECK(pid.compute(4990) == 2010);
    }

    TEST_CASE("init() resets the integrator") {
        PID pid(1000, 0, 1 << qn, 0, qn, feedbackPositive);
        pid.init(1000);
        CHECK(pid.compute(900) == midscale + 100);
        pid.init(1000);
        CHECK(pid.getIntegratorError() == 0);
        CHECK(pid.compute(1000) == midscale);

        SUBCASE("to a given value") {
            pid.init(1000, 5 << qn);
            CHECK(pid.getIntegratorError() == 5 << qn);
            CHECK(pid.compute(1000) == midscale + 5);
            pid.init(1000, -(5 << qn));
            CHECK(pid.compute(1000) == midscale - 5);
        }
        SUBCASE("which is limited to the output limits") {
            pid.setOutputMin(2000);
            pid.setOutputMax(2100);
            pid.init(1000, INT32_MAX);
            CHECK(pid.compute(1000) == 2100);
            pid.init(1000, INT32_MIN);
            CHECK(pid.compute(1000) == 2000);
            pid.init(1000, 10 << qn);
            CHECK(pid.compute(1000) == midscale + 10);
        }
        SUBCASE("even if zero is outside the output limits") {
            pid.setOutputMin(2100);
            pid.setOutputMax(2200);
            pid.init(1000);
            CHECK(pid.compute(1000) == 2100);
            pid.setOutputMin(1900);
            pid.setOutputMax(2000);
            pid.init(1000);
            CHECK(pid.compute(1000) == 2000);
        }
    }

    TEST_CASE("updateOutput() makes the transition from manual control bumpless") {
        PID pid(1000, 1 << qn, 1 << (qn - 3), 0, qn, feedbackPositive);
        pid.setOutputMin(500);
        pid.setOutputMax(3500);

        SUBCASE("the controller continues from the manual output") {
            pid.init(1000);
            pid.updateOutput(1234);
            CHECK(pid.compute(1000) == 1234);
            CHECK(pid.compute(1000) == 1234);
            pid.updateOutput(0x800);
            CHECK(pid.getIntegratorError() == 0);
        }
        SUBCASE("a manual output outside of the limits is limited") {
            pid.init(1000);
            pid.updateOutput(4000);
            CHECK(pid.compute(1000) == 3500);
            pid.updateOutput(100);
            CHECK(pid.compute(1000) == 500);
        }
    }
}

TEST_SUITE("extreme inputs") {
    TEST_CASE("the error is the difference of two unsigned numbers") {
        // ki = 1 and qn = 0: The integrator is the error and the output is the error in offset binary
        struct {
            uint32_t setpoint;
            uint32_t input;
            int32_t error;
        } const testVectors[] = {
            {0, 0, 0},
            {1000, 900, 100},
            {900, 1000, -100},
            {0x7FFFFFFFu, 0, INT32_MAX},
            {0, 0x7FFFFFFFu, -INT32_MAX},
            // Both values in the upper half of the range
            {0xC0000000u, 0xBFFFFFFFu, 1},
            {0xBFFFFFFFu, 0xC0000000u, -1},
            {UINT32_MAX, 0x80000000u, INT32_MAX},
            // The setpoint and the input are on different sides of midscale
            {0x80000000u, 0x7FFFFFFFu, 1},
            {0x7FFFFFFFu, 0x80000000u, -1},
            {0x80000064u, 0x7FFFFF9Cu, 200},
            {0x7FFFFF9Cu, 0x80000064u, -200},
            // The difference does not fit into a signed 32 bit number and saturates
            {0x80000000u, 0, INT32_MAX},
            {0, 0x80000000u, INT32_MIN},
            {UINT32_MAX, 0, INT32_MAX},
            {0, UINT32_MAX, INT32_MIN},
            {0xC0000000u, 0x10000000u, INT32_MAX},
            {0x10000000u, 0xC0000000u, INT32_MIN},
        };
        for (const auto& testVector : testVectors) {
            CAPTURE(testVector.setpoint);
            CAPTURE(testVector.input);
            PID pid(testVector.setpoint, 0, 1, 0, 0, feedbackPositive);
            pid.init(testVector.input);
            CHECK(pid.compute(testVector.input) == (uint32_t)testVector.error + MIDSCALE);
            CHECK(pid.getIntegratorError() == testVector.error);
        }
    }

    TEST_CASE("the derivative is the difference of two unsigned numbers") {
        // kd = 1 and qn = 0: The output is the negative change of the input in offset binary
        struct {
            uint32_t previousInput;
            uint32_t input;
            int32_t dInputNegative;
        } const testVectors[] = {
            {1000, 900, 100},
            {900, 1000, -100},
            {0xC0000000u, 0xBFFFFFFFu, 1},
            {0x80000000u, 0x7FFFFFFFu, 1},
            {0x7FFFFFFFu, 0x80000000u, -1},
            {0x7FFFFF9Cu, 0x80000064u, -200},
            {UINT32_MAX, 0, INT32_MAX},
            {0, UINT32_MAX, INT32_MIN},
            {0, 0x80000000u, INT32_MIN},
        };
        for (const auto& testVector : testVectors) {
            CAPTURE(testVector.previousInput);
            CAPTURE(testVector.input);
            PID pid(0, 0, 0, 1, 0, feedbackPositive);
            pid.init(testVector.previousInput);
            CHECK(pid.compute(testVector.input) == (uint32_t)testVector.dInputNegative + MIDSCALE);
        }
    }

    TEST_CASE("a controller regulating around midscale of a 32 bit input does not change its sign") {
        // A bipolar 32 bit ADC in offset binary notation outputs 0x80000000 for 0 V
        PID pid(0x80000000u, 1 << 8, 0, 0, 8, feedbackPositive);
        pid.init(0x80000000u);
        const uint32_t midscale = 1u << 23;
        CHECK(pid.compute(0x80000000u) == midscale);
        CHECK(pid.compute(0x80000000u - 1) == midscale + 1);
        CHECK(pid.compute(0x80000000u + 1) == midscale - 1);
        CHECK(pid.compute(0x80000000u - 1000) == midscale + 1000);
        CHECK(pid.compute(0x80000000u + 1000) == midscale - 1000);
    }
}

TEST_SUITE("reference models") {
    TEST_CASE("the controller matches the exact integer model for random gains and inputs") {
        std::mt19937 generator(0xC0FFEE);
        std::uniform_int_distribution<uint32_t> anyValue(0, UINT32_MAX);
        std::uniform_int_distribution<int> anyQn(0, 31);
        std::uniform_int_distribution<int> anyShift(0, 31);
        std::uniform_int_distribution<int> percent(0, 99);

        for (int run = 0; run < 2000; run++) {
            const uint8_t qn = (uint8_t)anyQn(generator);
            // Gains of every magnitude, the sign is determined by the feedback direction
            const int32_t kp = (int32_t)(anyValue(generator) >> anyShift(generator)) & INT32_MAX;
            const int32_t ki = (int32_t)(anyValue(generator) >> anyShift(generator)) & INT32_MAX;
            const int32_t kd = (int32_t)(anyValue(generator) >> anyShift(generator)) & INT32_MAX;
            const bool positive = percent(generator) < 50;
            const bool onError = percent(generator) < 50;
            // The inputs are either spread over the full range or confined to a window of 2^inputBits around an offset
            const int inputShift = anyShift(generator);
            const uint32_t inputOffset = percent(generator) < 50 ? 0 : anyValue(generator);
            const uint32_t setpoint = inputOffset + (anyValue(generator) >> inputShift);
            const int64_t sign = positive ? 1 : -1;

            PID pid(setpoint, kp, ki, kd, qn, positive ? feedbackPositive : feedbackNegative, onError ? proportionalToError : proportionalToInput);
            ReferencePid reference(setpoint, sign * kp, sign * ki, sign * kd, qn, onError);

            if (percent(generator) < 50) {
                // Limits which are a multiple of the output LSB, with min <= max
                const uint32_t outputBits = 32 - qn;
                const uint32_t outputMask = (uint32_t)(((uint64_t)1 << outputBits) - 1);
                uint32_t outputMin = anyValue(generator) & outputMask;
                uint32_t outputMax = anyValue(generator) & outputMask;
                if (outputMin > outputMax) {
                    std::swap(outputMin, outputMax);
                }
                pid.setOutputMin(outputMin);
                pid.setOutputMax(outputMax);
                reference.setOutputMin(outputMin);
                reference.setOutputMax(outputMax);
                // The integrator starts at 0 (midscale), which is limited by init()
                pid.init(0);
                reference.errorSum = limit(0, reference.outputMin, reference.outputMax);
            }

            CAPTURE(run);
            CAPTURE((int)qn);
            CAPTURE(kp);
            CAPTURE(ki);
            CAPTURE(kd);
            CAPTURE(positive);
            CAPTURE(onError);
            CAPTURE(setpoint);
            for (int i = 0; i < 100; i++) {
                const uint32_t input = inputOffset + (anyValue(generator) >> inputShift);
                CAPTURE(i);
                CAPTURE(input);
                REQUIRE(pid.compute(input) == reference.compute(input));
                REQUIRE(pid.getIntegratorError() == reference.errorSum);
            }
        }
    }

    /*
     * A textbook PID controller using double precision floating point numbers in units of the input and the output.
     * The derivative acts on the measurement and the integrator is limited to the output range.
     */
    struct DoublePid {
        double kp, ki, kd;
        double outputMin, outputMax;
        double integral;
        double previousInput;

        double compute(const double setpoint, const double input) {
            const double error = setpoint - input;
            this->integral = std::fmin(std::fmax(this->integral + this->ki * error, this->outputMin), this->outputMax);
            const double output = this->kp * error + this->integral - this->kd * (input - this->previousInput);
            this->previousInput = input;
            return std::fmin(std::fmax(output, this->outputMin), this->outputMax);
        }
    };

    TEST_CASE("the closed loop step response matches a double precision controller within one LSB") {
        // A 16 bit ADC and a 12 bit DAC. The gains are in units of DAC LSB per ADC LSB.
        const uint8_t qn = 20;
        const double scale = (double)(1u << qn);
        const double kp = 0.35;
        const double ki = 0.02;
        const double kd = 0.6;
        const double dacMin = 100.0;
        const double dacMax = 4000.0;
        const double dacMidscale = 2048.0;

        PID pid(20000, (int32_t)std::lround(kp * scale), (int32_t)std::lround(ki * scale), (int32_t)std::lround(kd * scale), qn, feedbackPositive);
        pid.setOutputMin((uint32_t)dacMin);
        pid.setOutputMax((uint32_t)dacMax);
        // The double precision controller uses the gains after quantisation. Its output is relative to midscale.
        DoublePid reference = {
            (double)pid.getKp() / scale, (double)pid.getKi() / scale, (double)pid.getKd() / scale,
            dacMin - dacMidscale, dacMax - dacMidscale, 0.0, 0.0,
        };

        // The plant is a first order low pass with a gain of 10 ADC LSB per DAC LSB and a time constant of 20 samples
        double plant = 8000.0;
        uint32_t setpoint = 20000;
        uint32_t input = (uint32_t)plant;
        pid.init(input);
        reference.previousInput = input;

        bool wasSaturatedHigh = false;
        bool wasSaturatedLow = false;
        uint32_t output = 0;
        for (int i = 0; i < 3000; i++) {
            if (i == 1000) {
                setpoint = 4000;    // Step down, which drives the output into the lower limit
                pid.setSetpoint(setpoint);
            }
            if (i == 2000) {
                setpoint = 30000;
                pid.setSetpoint(setpoint);
            }
            output = pid.compute(input);
            const double expected = reference.compute(setpoint, input) + dacMidscale;
            CAPTURE(i);
            CAPTURE(input);
            // The library truncates the fractional bits of the output, hence the tolerance of one LSB.
            REQUIRE((double)output <= expected + 1e-6);
            REQUIRE((double)output > expected - 1.0 - 1e-6);
            REQUIRE(output >= (uint32_t)dacMin);
            REQUIRE(output <= (uint32_t)dacMax);
            wasSaturatedHigh = wasSaturatedHigh || (output == (uint32_t)dacMax);
            wasSaturatedLow = wasSaturatedLow || (output == (uint32_t)dacMin);

            plant += (10.0 * output - plant) / 20.0;
            input = (uint32_t)std::lround(plant);

            // The loop has settled at the end of each section
            if (i == 999 || i == 2999) {
                CHECK(input == setpoint);
            }
            if (i == 1999) {
                CHECK(input == 4000);
            }
        }
        // The test is only meaningful if the limits and the anti-windup were exercised
        CHECK(wasSaturatedHigh);
        CHECK(wasSaturatedLow);
    }

    TEST_CASE("the open loop response to a noisy ramp matches a double precision controller within one LSB") {
        const uint8_t qn = 16;
        const double scale = (double)(1u << qn);
        std::mt19937 generator(42);
        std::uniform_int_distribution<int> noise(-50, 50);

        for (const FeedbackDirection direction : {feedbackNegative, feedbackPositive}) {
            const double sign = (direction == feedbackPositive) ? 1.0 : -1.0;
            PID pid(100000, 1234, 56, 7890, qn, direction);
            DoublePid reference = {sign * 1234 / scale, sign * 56 / scale, sign * 7890 / scale, -32768.0, 32767.0 + 65535.0 / 65536.0, 0.0, 90000.0};
            pid.init(90000);
            for (int i = 0; i < 5000; i++) {
                const uint32_t input = (uint32_t)(90000 + 4 * i + noise(generator));
                const uint32_t output = pid.compute(input);
                const double expected = reference.compute(100000.0, input) + 32768.0;
                CAPTURE(i);
                REQUIRE((double)output <= expected + 1e-6);
                REQUIRE((double)output > expected - 1.0 - 1e-6);
            }
        }
    }
}
