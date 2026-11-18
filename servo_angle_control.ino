/***************************************************
  Servo angle control for the Adafruit 16-channel PWM & Servo driver
  (PCA9685). Type an angle in the Serial Monitor and the servo moves
  to that angle.

  Based on the Adafruit servo example:
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Default I2C address 0x40. Use Adafruit_PWMServoDriver(0x41) if you changed it.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse range in counts (out of 4096). Tweak per servo so that 0 and 180
// degrees reach the ends of travel WITHOUT hitting the hard stop (a buzzing
// or hot servo means the range is too wide).
#define SERVOMIN   100   // pulse count at 0 degrees   (~0.6 ms)
#define SERVOMAX   550   // pulse count at 180 degrees (~2.4 ms)
#define SERVO_FREQ 50    // Analog servos run at ~50 Hz updates

// Angle range of your servo (use 270 for 270-degree servos, etc.)
#define ANGLE_MIN  0
#define ANGLE_MAX  180

#define PULSE_HARD_MIN 0
#define PULSE_HARD_MAX 1000

// Per-channel trim in degrees (positive = shifts motor toward higher pulse).
// All channels share the same SERVOMIN/SERVOMAX.
float angleOffset[16] = {0, 0, 0, 18.5, 0, 19.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // all zero by default

// Convert a degree angle to a PCA9685 pulse count
uint16_t angleToPulse(float angle, uint8_t channel) {
  angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);   // user-facing range
  angle += angleOffset[channel];                    // apply trim
  float pulse = SERVOMIN + (angle - ANGLE_MIN) * (SERVOMAX - SERVOMIN) / (ANGLE_MAX - ANGLE_MIN);
  return (uint16_t)constrain(pulse, PULSE_HARD_MIN, PULSE_HARD_MAX);
}

// Move a servo on a given channel to an angle in degrees
void setServoAngle(uint8_t channel, float angle) {
  uint16_t pulse = angleToPulse(angle, channel);
  pwm.setPWM(channel, 0, pulse);
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.print(" deg (pulse ");
  Serial.print(pulse);
  Serial.println(")");
}

void setup() {
  Serial.begin(9600);
  Serial.println("PCA9685 servo angle control");
  Serial.println("Enter an angle (e.g. 90) to move channel 3,");
  Serial.println("or 'channel angle' (e.g. 5 45) to move another channel.");

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  for (float i = 0; i < 16; i++) {
    setServoAngle(i, 0);  // start centered
  }
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    int space = line.indexOf(' ');
    uint8_t channel;
    float angle;

    channel = (uint8_t)line.substring(0, space).toInt();  // "5 45"
    angle = line.substring(space + 1).toFloat();

    if (channel > 15) {
      Serial.println("Channel must be 0-15.");
      return;
    }
    if (angle < ANGLE_MIN || angle > ANGLE_MAX) {
      Serial.print("Angle must be between ");
      Serial.print(ANGLE_MIN);
      Serial.print(" and ");
      Serial.println(ANGLE_MAX);
      return;
    }
    setServoAngle(channel, angle);
  }
}
