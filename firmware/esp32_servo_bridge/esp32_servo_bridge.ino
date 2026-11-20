// ESP32 servo bridge
//
// Reads one line of 12 pulse widths (microseconds) from the Pi over USB
// and sends them to the PCA9685, which drives the 12 servos.
//
//   Pi --USB--> ESP32 --I2C--> PCA9685 --> servos
//
// Message from the Pi (one line, numbers separated by spaces):
//   1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500
//
// Number i goes to PCA9685 channel i, in the same order as
// kinematics.ACTUATOR_ORDER:
//   0-2  leg a: hip_abduction, hip_pitch, knee
//   3-5  leg b: hip_abduction, hip_pitch, knee
//   6-8  leg c: hip_abduction, hip_pitch, knee
//   9-11 leg d: hip_abduction, hip_pitch, knee
//
// Wiring (ESP32 -> PCA9685): 3V3 -> VCC, GND -> GND, GPIO21 -> SDA, GPIO22 -> SCL
// Servo power comes from the UBEC, not the ESP32.
//
// Libraries: Wire (built in), "Adafruit PWM Servo Driver Library" (Library Manager)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int NUM_SERVOS = 12;
const int MIN_PULSE = 500;   // DS32xx servo range, in microseconds
const int MAX_PULSE = 2500;

Adafruit_PWMServoDriver pca;

void setup() {
  Serial.begin(115200);               // must match the baud rate in the Pi's code
  Wire.begin(21, 22);                 // I2C: SDA = GPIO21, SCL = GPIO22
  pca.begin();
  pca.setOscillatorFrequency(27000000);  // PCA9685 clock; tune if pulses measure off
  pca.setPWMFreq(50);                 // 50 pulses per second
  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) {
    return;                           // nothing from the Pi yet
  }

  // 1. Read one message from the Pi.
  String line = Serial.readStringUntil('\n');

  // 2. Pull the 12 numbers out of it.
  int pulses[NUM_SERVOS];
  const char* text = line.c_str();
  for (int i = 0; i < NUM_SERVOS; i++) {
    char* end;
    pulses[i] = strtol(text, &end, 10);
    if (end == text) {                // fewer than 12 numbers: ignore the message
      Serial.println("ERR expected 12 numbers");
      return;
    }
    text = end;
  }

  // 3. Send each one to its servo, kept inside the safe range.
  for (int i = 0; i < NUM_SERVOS; i++) {
    pca.writeMicroseconds(i, constrain(pulses[i], MIN_PULSE, MAX_PULSE));
  }
}
