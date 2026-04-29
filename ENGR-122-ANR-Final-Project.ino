#include "MQTT.h"
#include <SSD1306Wire.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include <math.h>

SSD1306Wire display(0x3C, D2, D1);

#define MOTOR_LEFT_PIN  D3
#define MOTOR_RIGHT_PIN D6

Servo motorLeft;
Servo motorRight;


Ultrasonic us_left  (TX, D7);
Ultrasonic us_center(D8, D0);
Ultrasonic us_right (D4, D5);

// four targets and locations
const int NUM_TARGETS  = 4;
const int x_targets[]  = {695, 1541, 2087, 133};
const int y_targets[]  = {680,  142,  625, 180};
int target_index       = 0;

const char* mqtt_server   = "192.168.0.2";
const char* mqtt_username = "user";
const char* mqtt_password = "Stevens1870";
const int   mqtt_port     = 1883;

const String topic   = "EAS011_North";
const char* ssid     = "TP-Link_8FFD";
const char* password = "68287078";

float x_robot, y_robot, z_ang_robot;

const int MOTOR_STOP      = 90;
const int SPEED_BASE      = 117;
const int SPEED_SLOW      = 105;
const int SPEED_SPIN_FAST = 120;
const int SPEED_SPIN_SLOW = 60;

const float ANGLE_DEADBAND  = 15.0;
const float TARGET_RADIUS   = 140.0;

const int OBS_FRONT_TRIGGER = 120;
const int OBS_SIDE_CLOSE    = 180;  

enum RobotState {
  NAVIGATE,
  AVOID_OBSTACLE,
  AT_TARGET,
  ALL_DONE
};

RobotState currentState = NAVIGATE;

unsigned long avoidStartTime     = 0;
const unsigned long AVOID_TURN_MS = 450;
const unsigned long AVOID_FWD_MS  = 600;

unsigned long targetReachedTime   = 0;
const unsigned long TARGET_PAUSE_MS = 2000;


void setMotors(int leftVal, int rightVal) {
  motorLeft.write(leftVal);
  motorRight.write(rightVal);
}

// distance formula for next target
float distanceToTarget() {
  float dx = x_targets[target_index] - x_robot;
  float dy = y_targets[target_index] - y_robot;
  return sqrt(dx * dx + dy * dy);
}
// optimal angle to next target
float computeAngleError() {
  float dx = x_targets[target_index] - x_robot;
  float dy = y_targets[target_index] - y_robot;
  float target_angle = atan2(dy, dx) * 180.0 / M_PI;
  float error = target_angle - z_ang_robot;
  while (error >  180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}

void handleNavigate(int distL, int distC, int distR) {

// swtich to avoid obstacle mode when close to walls
  if (distC <= OBS_FRONT_TRIGGER ||
     (distL <= OBS_SIDE_CLOSE && distR <= OBS_SIDE_CLOSE)) {
    setMotors(MOTOR_STOP, MOTOR_STOP);
    avoidStartTime = millis();
    currentState = AVOID_OBSTACLE;
    return;
  }
// pause at target
  if (distanceToTarget() <= TARGET_RADIUS) {
    setMotors(MOTOR_STOP, MOTOR_STOP);
    targetReachedTime = millis();
    currentState = AT_TARGET;
    return;
  }

  float error = computeAngleError();

  // drift opposite direction when close to wall
  if (distL <= OBS_SIDE_CLOSE && distC > OBS_FRONT_TRIGGER) {
    setMotors(SPEED_BASE, SPEED_SLOW);
    return;
  }
  if (distR <= OBS_SIDE_CLOSE && distC > OBS_FRONT_TRIGGER) {
    setMotors(SPEED_SLOW, SPEED_BASE);
    return;
  }

  // normal angle-error steering
  if (error > ANGLE_DEADBAND) {
    setMotors(SPEED_BASE, SPEED_SLOW);
  } else if (error < -ANGLE_DEADBAND) {
    setMotors(SPEED_SLOW, SPEED_BASE);
  } else {
    setMotors(SPEED_BASE, SPEED_BASE);
  }
}

void handleAvoidObstacle(int distL, int distC, int distR) {
  unsigned long elapsed = millis() - avoidStartTime;

  if (elapsed < AVOID_TURN_MS) {
    // 8 different decisions
    if (distC <= OBS_FRONT_TRIGGER && distL <= OBS_SIDE_CLOSE && distR <= OBS_SIDE_CLOSE) {
      // blocked on all three sides, reverse
      setMotors(SPEED_SPIN_SLOW, SPEED_SPIN_SLOW);
    } else if (distL >= distR) {
      setMotors(SPEED_SPIN_SLOW, SPEED_SPIN_FAST); // more room left, turn left
    } else {
      setMotors(SPEED_SPIN_FAST, SPEED_SPIN_SLOW); // more room right, turn right
    }
  } else if (elapsed < AVOID_TURN_MS + AVOID_FWD_MS) {
    // check sides when driving  drift if needed
    if (distL <= OBS_SIDE_CLOSE && distR > OBS_SIDE_CLOSE) {
      setMotors(SPEED_BASE, SPEED_SLOW); // drift right to avoid left wall
    } else if (distR <= OBS_SIDE_CLOSE && distL > OBS_SIDE_CLOSE) {
      setMotors(SPEED_SLOW, SPEED_BASE); // drift left to avoid right wall
    } else {
      setMotors(SPEED_SLOW, SPEED_SLOW); // good on both sides go straight slow 
    }
  } else {
    currentState = NAVIGATE; // drive straight 
  }

  if (elapsed >= AVOID_TURN_MS && distC <= OBS_FRONT_TRIGGER) {
    avoidStartTime = millis();
  }
}
// target navigation order 
void handleAtTarget() {
  setMotors(MOTOR_STOP, MOTOR_STOP);
  if (millis() - targetReachedTime >= TARGET_PAUSE_MS) {
    target_index++;
    if (target_index >= NUM_TARGETS) {
      currentState = ALL_DONE;
    } else {
      currentState = NAVIGATE;
    }
  }
}

// OLED setup

void setup() {
  Wire.begin();
  delay(25);
  wifi_mqtt_init();
  mqtt_clean();
  delay(500);

  display.init();
  display.flipScreenVertically();

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0,  "ENGR122 Final Project");
  display.drawString(0, 16, "Connecting...");
  display.display();

  motorLeft.attach(MOTOR_LEFT_PIN);
  motorRight.attach(MOTOR_RIGHT_PIN);
  setMotors(MOTOR_STOP, MOTOR_STOP);

  delay(3000);

  display.clear();
  display.drawString(0, 0, "Ready! Starting...");
  display.display();
  delay(500);
}

void loop() {
  unsigned long stamp = millis();

  mqtt_rebound();
  client.loop();

  int distLeft   = us_left.read(CM) * 10;
  int distCenter = us_center.read(CM) * 10;
  int distRight  = us_right.read(CM) * 10;

  float dist     = (target_index < NUM_TARGETS) ? distanceToTarget() : 0;
  float angleErr = (target_index < NUM_TARGETS) ? computeAngleError() : 0;

  switch (currentState) {
    case NAVIGATE:
      handleNavigate(distLeft, distCenter, distRight);
      break;
    case AVOID_OBSTACLE:
      handleAvoidObstacle(distLeft, distCenter, distRight);
      break;
    case AT_TARGET:
      handleAtTarget();
      break;
    case ALL_DONE:
      setMotors(MOTOR_STOP, MOTOR_STOP);
      break;
  }

  // OLED display 
  display.clear();
  display.setFont(ArialMT_Plain_10);

  String stateName;
  switch (currentState) {
    case NAVIGATE:       stateName = "NAV";    break;
    case AVOID_OBSTACLE: stateName = "AVOID";  break;
    case AT_TARGET:      stateName = "AT TGT"; break;
    case ALL_DONE:       stateName = "DONE";   break;
  }

  display.drawString(0,  0, "X:" + String((int)x_robot) +
                             " Y:" + String((int)y_robot));
  display.drawString(0, 12, "Hdg:" + String((int)z_ang_robot) +
                             " St:" + stateName);
  display.drawString(0, 24, "Dist:" + String((int)dist) +
                             " Err:" + String((int)angleErr));
  display.drawString(0, 36, "L:" + String(distLeft) +
                             " C:" + String(distCenter) +
                             " R:" + String(distRight));
  display.drawString(0, 48, "Tgt#" + String(target_index + 1) +
                             " (" + String(x_targets[target_index]) +
                             "," + String(y_targets[target_index]) + ")");
  display.display();

  mqtt_clean();
  while (millis() - stamp < 100) { delay(5); }
}