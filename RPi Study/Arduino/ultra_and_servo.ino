#include <Servo.h>

#define SERVO_PIN 9
#define TRIG_PIN 7
#define ECHO_PIN 8

Servo scanner;

// Sweep parameters
const int MIN_ANGLE = 10;
const int MAX_ANGLE = 170;
const unsigned long CYCLE_MS = 4000;           // 4초에 0→180→0 한 주기
const int TOTAL_STEPS  = (MAX_ANGLE - MIN_ANGLE) * 2;  
const unsigned int STEP_DELAY_US = 
    CYCLE_MS * 1000UL / TOTAL_STEPS;           // 약 11111µs

void setup() {
  Serial.begin(9600);           // initialize serial at 9600 baud
  scanner.attach(SERVO_PIN);    // attach servo signal pin
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // sweep up: 0 -> 180
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle++) {
    measureAndSend(angle);
    delayMicroseconds(STEP_DELAY_US);
  }
  // sweep down: 180 -> 0
  for (int angle = MAX_ANGLE; angle >= MIN_ANGLE; angle--) {
    measureAndSend(angle);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void measureAndSend(int angle) {
  scanner.write(angle);    // move servo to 'angle'
  delay(10);               // allow servo to settle (tweak as needed)

  // trigger ultrasonic burst
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // read echo pulse (timeout 20 ms)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  float distance;
  if (duration == 0) {
    distance = -1;         // no echo within timeout
  } else {
    distance = (duration * 0.0343) / 2.0;  // convert µs to cm
  }

  // send "angle,distance"
  Serial.print(angle);
  Serial.print(',');
  Serial.println(distance);
}
