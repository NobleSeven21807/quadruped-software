// ESP32 single-servo test
//
// Reads one pulse width (microseconds) per line from the Pi over USB
// and sends it to one servo on the PCA9685.
//
//   Pi --USB--> ESP32 --I2C--> PCA9685 --> 1 servo
//
// Message from the Pi (one number per line):
//   1500
//
// Wiring (ESP32 -> PCA9685): 3V3 -> VCC, GND -> GND, GPIO21 -> SDA, GPIO22 -> SCL
// Servo plugs into PCA9685 channel SERVO_CHANNEL. Servo power comes from the UBEC.
//
// Libraries: Wire (built in), "Adafruit PWM Servo Driver Library" (Library Manager)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int SERVO_CHANNEL = 0;  // which PCA9685 channel the servo is plugged into
const int MIN_PULSE = 500;    // DS32xx servo range, in microseconds
const int MAX_PULSE = 2500;

Adafruit_PWMServoDriver pca;

void setup() {
  Serial.begin(115200);                  // must match the baud rate in the Pi's code
  Wire.begin(21, 22);                    // I2C: SDA = GPIO21, SCL = GPIO22
  pca.begin();
  pca.setOscillatorFrequency(27000000);  // PCA9685 clock; tune if pulses measure off
  pca.setPWMFreq(50);                    // 50 pulses per second
  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) {
    return;                              // nothing from the Pi yet
  }

  int pulse = Serial.parseInt();         // read the number the Pi sent
  Serial.readStringUntil('\n');          // throw away the rest of the line

  if (pulse == 0) {
    return;                              // blank or non-number line: ignore it
  }

  pulse = constrain(pulse, MIN_PULSE, MAX_PULSE);  // keep it in the safe range
  pca.writeMicroseconds(SERVO_CHANNEL, pulse);
  Serial.println(pulse);                 // echo back so you can see it worked
}
