#include <Servo.h>

const int SERVO_COUNT = 5;
const uint8_t servoPins[SERVO_COUNT] = {2, 3, 4, 5, 6};
Servo servos[SERVO_COUNT];

const int ANGLE_MIN = 5;
const int ANGLE_MAX = 175;

int currentAngle[SERVO_COUNT];

int clampAngle(int a) {
  if (a < ANGLE_MIN) return ANGLE_MIN;
  if (a > ANGLE_MAX) return ANGLE_MAX;
  return a;
}

void smoothMoveTo(int idx, int targetAngle, int stepDelay=15, int stepSize=1) {
  targetAngle = clampAngle(targetAngle);
  int cur = currentAngle[idx];
  if (cur == targetAngle) return;
  if (cur < targetAngle) {
    for (int a = cur; a <= targetAngle; a += stepSize) {
      servos[idx].write(a);
      delay(stepDelay);
    }
  } else {
    for (int a = cur; a >= targetAngle; a -= stepSize) {
      servos[idx].write(a);
      delay(stepDelay);
    }
  }
  currentAngle[idx] = targetAngle;
}

String inputBuffer = "";
bool isTracking = false;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < SERVO_COUNT; ++i) {
    servos[i].attach(servoPins[i]);
    currentAngle[i] = 90;
    servos[i].write(currentAngle[i]);
    delay(50);
  }
  Serial.println("READY");
}

void initSequence() {
  smoothMoveTo(4, 0, 15, 1);   // D6 -> 0
  smoothMoveTo(3, 0, 15, 1);    // D5 -> 0
  smoothMoveTo(2, 90, 15, 1);   // D4 -> 90
  smoothMoveTo(1, 35, 15, 1);   // D3 -> 35
  smoothMoveTo(0, 90, 15, 1);   // D2 -> 90
  Serial.println("DONE");
}

void searchMode() {
  smoothMoveTo(4, 0, 15, 1);   // D6 -> 0
  smoothMoveTo(3, 10, 15, 1);   // D5 -> 10
  smoothMoveTo(2, 10, 15, 1);   // D4 -> 10
  smoothMoveTo(1, 15, 15, 1);   // D3 -> 15
  isTracking = true;
  Serial.println("DONE");
}

void stopMode() {
  isTracking = false;
  for (int i = 0; i < SERVO_COUNT; ++i) {
    smoothMoveTo(i, 90, 15, 1);
  }
  Serial.println("DONE");
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("1")) {
    initSequence();
    return;
  }
  
  if (cmd.equalsIgnoreCase("2")) {
    searchMode();
    return;
  }
  
  if (cmd.equalsIgnoreCase("3")) {
    stopMode();
    return;
  }
  
  if (isTracking && cmd.charAt(0) == 'S') {
    int colon = cmd.indexOf(':');
    if (colon > 1) {
      String idxStr = cmd.substring(1, colon);
      String angStr = cmd.substring(colon + 1);
      int idx = idxStr.toInt();
      int ang = angStr.toInt();
      if (idx >= 0 && idx < SERVO_COUNT) {
        ang = clampAngle(ang);
        smoothMoveTo(idx, ang, 12, 1);
        Serial.print("OK ");
        Serial.print(idx);
        Serial.print(":");
        Serial.println(ang);
      } else {
        Serial.println("ERR_IDX");
      }
    } else {
      Serial.println("ERR_FMT");
    }
    return;
  }
  Serial.println("UNKNOWN");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > 120) inputBuffer = inputBuffer.substring(inputBuffer.length() - 120);
    }
  }
}