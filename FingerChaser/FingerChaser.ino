#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#define LLangle 65     // lower limit of angular range
#define ULangle 160    // upper limit of angular range
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates
#define SERVOMIN 80    // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 530   // This is the 'maximum' pulse length count (out of 4096)
// the difference is 450 devided by 180° results in 2.5 steps per degree for the angle

VL53L0X sensor;
int distance, pos, posold, posveryold;  // Distance an servo positions in steps (= 0.4°)
int angle, startangle;                  // angle and angle of first detecting the obstacle (startangle)

byte counter = 1;  // switch counter
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    delay(1000);
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous(20);
  angle = LLangle;
  pwm.setPWM(12, 0, SERVOMIN + (int)(angle * 2.5));
  Serial.println(angle);
}

void loop() {
  switch (counter) {
    case 1:
      {  // return to lower limit
        angle = LLangle;
        pwm.setPWM(12, 0, SERVOMIN + (int)(angle * 2.5));
        delay(100);
        distance = sensor.readRangeContinuousMillimeters();
        counter = 2;
        break;
      }
    case 2:
      {  // search obstacle begin by scanning
        if (angle > ULangle) {
          counter = 1;
          break;
        }

        pwm.setPWM(12, 0, SERVOMIN + (int)(angle * 2.5));
        distance = sensor.readRangeContinuousMillimeters();
        if (distance < 400) {
          startangle = angle;
          Serial.println("obstacle start at  " + (String)startangle + "  distance   " + (String)distance);
          pos = (int)(startangle * 2.5);
          posold = pos;
          pwm.setPWM(12, 0, SERVOMIN + pos);
          counter = 3;
          break;

        } else angle += 2;
        break;
      }
    case 3:
      {  // regulate at startangle
        distance = sensor.readRangeContinuousMillimeters();
        posveryold = posold;
        posold = pos;
        if (distance < 400) pos--;
        if (distance > 400) pos++;

        if (posveryold < posold && posold < pos) pos += 5;  // to speed up the follow
        if (posveryold > posold && posold > pos) pos -= 5;  // to speed up the follow

        angle = (int)(pos / 2.5);
        // recover if leave limits or out of lock
        if (angle < LLangle || angle > ULangle) counter = 1;
        pwm.setPWM(12, 0, SERVOMIN + pos);
        break;
      }
      break;
  }
}
