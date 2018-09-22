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
  digitalWrite(13, HIGH);

  pidController.setOutputMin(0);      // This is the default
  pidController.setOutputMax(1023);   // The Arduino has a 10-bit'analog' PWM output,
                                      // but the maximum output can be adjusted down.

  pidController.init(analogRead(ADC_INPUT_PIN));  // Initialize the pid controller to make sure there
                                      // are no output spikes
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
