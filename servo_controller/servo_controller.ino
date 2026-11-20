/***************************************************
  Servo angle control for the Adafruit 16-channel PWM & Servo driver
  (PCA9685). Type an angle in the Serial Monitor and the servo moves
  to that angle.
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

// Angle range of your servo (use 180 degrees for our servos)
#define ANGLE_MIN  0
#define ANGLE_MAX  180

#define PULSE_HARD_MIN 0
#define PULSE_HARD_MAX 1000

// Safety limits for ball balancer in degrees.
#define SAFETY_LIMIT_MIN 0
#define SAFETY_LIMIT_MAX 50

// Per-channel trim in degrees (positive = shifts motor toward higher pulse).
float angleOffset[16] = {0, 0, 0, 18.5, 0, 19.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // all zero by default

// Convert a degree angle to a PCA9685 pulse count
uint16_t angleToPulse(float angle, uint8_t channel) {
  angle = constrain(angle, SAFETY_LIMIT_MIN, SAFETY_LIMIT_MAX);
  angle += angleOffset[channel];                    // apply trim
  float pulse = SERVOMIN + (angle - ANGLE_MIN) * (SERVOMAX - SERVOMIN) / (ANGLE_MAX - ANGLE_MIN);
  return (uint16_t)constrain(pulse, PULSE_HARD_MIN, PULSE_HARD_MAX);
}

// Move a servo on a given channel to an angle in degrees
void setServoAngle(uint8_t channel, float angle) {
  uint16_t pulse = angleToPulse(angle, channel);
  pwm.setPWM(channel, 0, pulse);
  Serial.print("[ARDUINO] Channel ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.print(" deg (pulse ");
  Serial.print(pulse);
  Serial.println(")");
}

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 servo control: Enter a 'channel angle' (e.g. 5 45) to move another channel.");

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

    setServoAngle(channel, angle);
  }
}