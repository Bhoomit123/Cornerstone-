
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <math.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled for this ESP32 target"
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP is not available on this ESP32 target"
#endif

BluetoothSerial SerialBT;

// ==========================================
// FRSBOT - 12 DOF CONTINUOUS WALKING
// ESP32 DEVKIT V1
// ==========================================

Servo servos[12];

const int servoPins[12] = {
  13, 14, 27, 26, 33, 32,
  25, 19, 21, 18, 23, 22
};

// ==========================================
// MOVEMENT SETTINGS
// ==========================================

const unsigned long MOVE_DURATION_MS = 120;
const unsigned long FRAME_INTERVAL_MS = 10;
const unsigned long HOLD_TIME_MS = 1;

// ==========================================
// WALKING CONTROL
// ==========================================

char walkingMode = 'S';

bool stopRequested = false;
bool ignoreStopInput = false;

// Latest command received from the phone or USB Serial
char pendingCommand = 0;
bool commandPending = false;
bool bluetoothWasConnected = false;

// ==========================================
// NORMAL STANDING POSITIONS
// ==========================================

int normalStanding[12] = {
  115, 107, 130,
  120, 115, 50,
  50, 60, 65,
  130, 70, 70
};

// ==========================================
// FORWARD STARTING POSITIONS
// ==========================================

int forwardStarting[12] = {
  115, 107, 130,
  120, 115, 80,
  50, 60, 65,
  105, 70, 70
};

// ==========================================
// BACKWARD STARTING POSITIONS
// ==========================================

int backwardStarting[12] = {
  115, 107, 90,
  120, 115, 50,
  85, 60, 65,
  130, 70, 70
};

// ==========================================
// CURRENT SERVO POSITIONS
// ==========================================

int currentPositions[12] = {
  115, 107, 130,
  120, 115, 50,
  50, 60, 65,
  130, 70, 70
};

// ==========================================
// BLUETOOTH + USB COMMAND INPUT
// Commands: F, B, L, R, X, S
// ==========================================

char readControlCommand() {

  // Prefer commands from the phone; also keep USB Serial available.
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r' || c == ' ' || c == '\t') continue;
    c = toupper(c);
    if (c == 'F' || c == 'B' || c == 'L' ||
        c == 'R' || c == 'X' || c == 'S') return c;
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ' || c == '\t') continue;
    c = toupper(c);
    if (c == 'F' || c == 'B' || c == 'L' ||
        c == 'R' || c == 'X' || c == 'S') return c;
  }

  return 0;
}

void checkStopCommand() {

  if (ignoreStopInput) return;

  // Safety: if the phone disconnects while the robot is moving,
  // request a stop and return to normal standing.
  bool bluetoothConnected = SerialBT.hasClient();
  if (bluetoothWasConnected && !bluetoothConnected && walkingMode != 'S') {
    pendingCommand = 'X';
    commandPending = true;
    stopRequested = true;
    walkingMode = 'S';
    Serial.println("Bluetooth disconnected: stopping robot");
  }
  bluetoothWasConnected = bluetoothConnected;

  char command = readControlCommand();

  if (command != 0) {
    // Stop the current movement safely. The main loop will stand the
    // robot, then start the newly requested mode from normal standing.
    pendingCommand = command;
    commandPending = true;
    stopRequested = true;
    walkingMode = 'S';

    Serial.print("\nCOMMAND RECEIVED: ");
    Serial.println(command);
  }
}

// ==========================================
// SINGLE SERVO SMOOTH MOVEMENT
// ==========================================

bool moveServoSmooth(int servoIndex, int targetAngle) {

  if (servoIndex < 0 || servoIndex >= 12) return false;

  targetAngle = constrain(targetAngle, 0, 180);

  int startAngle = currentPositions[servoIndex];
  int lastAngle = startAngle;

  unsigned long startTime = millis();

  while (millis() - startTime < MOVE_DURATION_MS) {

    checkStopCommand();

    if (stopRequested) {

      servos[servoIndex].write(lastAngle);
      currentPositions[servoIndex] = lastAngle;

      return false;
    }

    float t = (float)(millis() - startTime) /
              MOVE_DURATION_MS;

    float smooth = 6 * pow(t, 5)
                 - 15 * pow(t, 4)
                 + 10 * pow(t, 3);

    lastAngle = startAngle +
                (targetAngle - startAngle) * smooth;

    servos[servoIndex].write(lastAngle);

    delay(FRAME_INTERVAL_MS);
  }

  servos[servoIndex].write(targetAngle);
  currentPositions[servoIndex] = targetAngle;

  return true;
}

// ==========================================
// TWO SERVO SYNCHRONIZED MOVEMENT
// ==========================================

bool moveTwoServosSmooth(
  int servo1, int target1,
  int servo2, int target2
) {

  if (servo1 < 0 || servo1 >= 12 ||
      servo2 < 0 || servo2 >= 12) return false;

  target1 = constrain(target1, 0, 180);
  target2 = constrain(target2, 0, 180);

  int start1 = currentPositions[servo1];
  int start2 = currentPositions[servo2];

  int last1 = start1;
  int last2 = start2;

  unsigned long startTime = millis();

  while (millis() - startTime < MOVE_DURATION_MS) {

    checkStopCommand();

    if (stopRequested) {

      servos[servo1].write(last1);
      servos[servo2].write(last2);

      currentPositions[servo1] = last1;
      currentPositions[servo2] = last2;

      return false;
    }

    float t = (float)(millis() - startTime) /
              MOVE_DURATION_MS;

    float smooth = 6 * pow(t, 5)
                 - 15 * pow(t, 4)
                 + 10 * pow(t, 3);

    last1 = start1 + (target1 - start1) * smooth;
    last2 = start2 + (target2 - start2) * smooth;

    servos[servo1].write(last1);
    servos[servo2].write(last2);

    delay(FRAME_INTERVAL_MS);
  }

  servos[servo1].write(target1);
  servos[servo2].write(target2);

  currentPositions[servo1] = target1;
  currentPositions[servo2] = target2;

  return true;
}

// ==========================================
// MOVE TO COMPLETE POSITION
// ==========================================

bool moveToPosition(int targetPositions[]) {

  for (int i = 0; i < 12; i++) {

    if (!moveServoSmooth(i, targetPositions[i])) {
      return false;
    }
  }

  return true;
}

// ==========================================
// NORMAL STANDING
// ==========================================

void standRobot() {

  Serial.println("\nRETURNING TO NORMAL STANDING");

  stopRequested = false;
  ignoreStopInput = true;

  moveToPosition(normalStanding);

  ignoreStopInput = false;

  Serial.println("NORMAL STANDING COMPLETE");
}

// ==========================================
// FORWARD PREPARATION
// RUN ONLY ONCE WHEN F IS RECEIVED
// ==========================================

bool prepareForward() {

  Serial.println("\nFORWARD START POSITION");

  // FIRST: S6 = 80 AND S10 = 105 TOGETHER
  if (!moveTwoServosSmooth(5, 80, 9, 105))
    return false;

  // REMAINING SERVOS
  for (int i = 0; i < 12; i++) {

    if (i == 5 || i == 9) continue;

    if (!moveServoSmooth(i, forwardStarting[i]))
      return false;
  }

  Serial.println("FORWARD POSITION READY");

  return true;
}

// ==========================================
// BACKWARD PREPARATION
// RUN ONLY ONCE WHEN B IS RECEIVED
// ==========================================

bool prepareBackward() {

  Serial.println("\nBACKWARD START POSITION");

  // FIRST: S3 = 90 AND S7 = 85 TOGETHER
  if (!moveTwoServosSmooth(2, 90, 6, 85))
    return false;

  // REMAINING SERVOS
  for (int i = 0; i < 12; i++) {

    if (i == 2 || i == 6) continue;

    if (!moveServoSmooth(i, backwardStarting[i]))
      return false;
  }

  Serial.println("BACKWARD POSITION READY");

  return true;
}

// ==========================================
// GENERIC LEG MOVEMENT
// ==========================================

bool moveLeg(
  int hipIndex, int hipTarget,
  int liftIndex, int liftTarget, int liftReturn,
  int footIndex, int footTarget, int footReturn
) {

  if (!moveServoSmooth(hipIndex, hipTarget))
    return false;

  delay(HOLD_TIME_MS);

  if (!moveServoSmooth(liftIndex, liftTarget))
    return false;

  delay(HOLD_TIME_MS);

  if (!moveServoSmooth(footIndex, footTarget))
    return false;

  delay(HOLD_TIME_MS);

  if (!moveServoSmooth(liftIndex, liftReturn))
    return false;

  delay(HOLD_TIME_MS);

  if (!moveServoSmooth(footIndex, footReturn))
    return false;

  return true;
}

// ==========================================
// FORWARD FRONT LEFT
// ==========================================

bool frontLeftForward() {

  return moveLeg(
    5, 35,
    4, 140, 115,
    3, 90, 120
  );
}

// ==========================================
// FORWARD BACK LEFT
// ==========================================

bool backLeftForward() {

  return moveLeg(
    2, 100,
    1, 137, 107,
    0, 95, 115
  );
}

// ==========================================
// FORWARD LEFT HIP RETURN
// ==========================================

bool leftHipsForwardReturn() {

  return moveTwoServosSmooth(5, 80, 2, 130);
}

// ==========================================
// FORWARD FRONT RIGHT
// ==========================================

bool frontRightForward() {

  return moveLeg(
    9, 145,
    10, 40, 70,
    11, 95, 75
  );
}

// ==========================================
// FORWARD BACK RIGHT
// ==========================================

bool backRightForward() {

  return moveLeg(
    6, 80,
    8, 40, 65,
    7, 80, 60
  );
}

// ==========================================
// FORWARD RIGHT HIP RETURN
// ==========================================

bool rightHipsForwardReturn() {

  return moveTwoServosSmooth(9, 105, 6, 50);
}

// ==========================================
// ONE FORWARD WALKING CYCLE
// NO STARTING POSITION HERE
// ==========================================

bool forwardWalkingCycle() {

  if (!frontLeftForward()) return false;
  if (!backLeftForward()) return false;
  if (!leftHipsForwardReturn()) return false;

  if (!frontRightForward()) return false;
  if (!backRightForward()) return false;
  if (!rightHipsForwardReturn()) return false;

  return true;
}

// ==========================================
// BACKWARD FRONT LEFT
// ==========================================

bool frontLeftBackward() {

  return moveLeg(
    5, 75,
    4, 140, 115,
    3, 90, 120
  );
}

// ==========================================
// BACKWARD BACK LEFT
// S3: 90 -> 130 -> 90
// ==========================================

bool backLeftBackward() {

  return moveLeg(
    2, 130,
    1, 137, 107,
    0, 95, 115
  );
}

// ==========================================
// BACKWARD LEFT HIP RETURN
// ==========================================

bool leftHipsBackwardReturn() {

  return moveTwoServosSmooth(5, 50, 2, 90);
}

// ==========================================
// BACKWARD FRONT RIGHT
// ==========================================

bool frontRightBackward() {

  return moveLeg(
    9, 105,
    10, 40, 70,
    11, 95, 75
  );
}

// ==========================================
// BACKWARD BACK RIGHT
// S7: 85 -> 45 -> 85
// ==========================================

bool backRightBackward() {

  return moveLeg(
    6, 45,
    8, 40, 65,
    7, 80, 60
  );
}

// ==========================================
// BACKWARD RIGHT HIP RETURN
// ==========================================

bool rightHipsBackwardReturn() {

  return moveTwoServosSmooth(9, 130, 6, 85);
}

// ==========================================
// ONE BACKWARD WALKING CYCLE
// NO STARTING POSITION HERE
// ==========================================

bool backwardWalkingCycle() {

  if (!frontLeftBackward()) return false;
  if (!backLeftBackward()) return false;
  if (!leftHipsBackwardReturn()) return false;

  if (!frontRightBackward()) return false;
  if (!backRightBackward()) return false;
  if (!rightHipsBackwardReturn()) return false;

  return true;
}

// ==========================================
// TURNING LEG FUNCTIONS
// ==========================================

// LEFT TURN - Front Left: S6 50 -> 25
bool frontLeftTurn() {
  return moveLeg(
    5, 25,
    4, 140, 115,
    3, 90, 120
  );
}

// LEFT TURN - Back Left: S3 130 -> 160
bool backLeftTurn() {
  return moveLeg(
    2, 160,
    1, 137, 107,
    0, 95, 115
  );
}

// Both turns - Front Right: S10 130 -> 155
bool frontRightTurn() {
  return moveLeg(
    9, 155,
    10, 40, 70,
    11, 95, 75
  );
}

// LEFT TURN - Back Right remains S7 50 -> 80
bool backRightTurnLeft() {
  return moveLeg(
    6, 80,
    8, 40, 65,
    7, 80, 60
  );
}

// RIGHT TURN - Back Left remains S3 130 -> 100
bool backLeftTurnRight() {
  return moveLeg(
    2, 100,
    1, 137, 107,
    0, 95, 115
  );
}

// RIGHT TURN - Back Right: S7 50 -> 20
bool backRightTurnRight() {
  return moveLeg(
    6, 20,
    8, 40, 65,
    7, 80, 60
  );
}

// Return the two hips on each side to normal standing.
bool leftTurnHipReturn() {
  return moveTwoServosSmooth(5, 50, 2, 130);
}

bool rightTurnHipReturn() {
  return moveTwoServosSmooth(9, 130, 6, 50);
}

// LEFT TURN: FL forward, BL backward, FR forward, BR forward
bool leftWalkingCycle() {

  if (!frontLeftTurn()) return false;
  if (!backLeftTurn()) return false;
  if (!leftTurnHipReturn()) return false;

  if (!frontRightTurn()) return false;
  if (!backRightTurnLeft()) return false;
  if (!rightTurnHipReturn()) return false;

  return true;
}

// RIGHT TURN: FL forward, BL forward, FR forward, BR backward
bool rightWalkingCycle() {

  if (!frontLeftTurn()) return false;
  if (!backLeftTurnRight()) return false;
  if (!leftTurnHipReturn()) return false;

  if (!frontRightTurn()) return false;
  if (!backRightTurnRight()) return false;
  if (!rightTurnHipReturn()) return false;

  return true;
}

// ==========================================
// EXECUTE A RECEIVED COMMAND
// ==========================================

void executeCommand(char command) {

  stopRequested = false;

  switch (command) {

    case 'F':
      Serial.println("\nFORWARD COMMAND");
      if (prepareForward()) {
        walkingMode = 'F';
        Serial.println("FORWARD WALKING STARTED");
      } else {
        walkingMode = 'S';
      }
      break;

    case 'B':
      Serial.println("\nBACKWARD COMMAND");
      if (prepareBackward()) {
        walkingMode = 'B';
        Serial.println("BACKWARD WALKING STARTED");
      } else {
        walkingMode = 'S';
      }
      break;

    case 'L':
      walkingMode = 'L';
      Serial.println("LEFT TURN STARTED");
      break;

    case 'R':
      walkingMode = 'R';
      Serial.println("RIGHT TURN STARTED");
      break;

    case 'X':
    case 'S':
      walkingMode = 'S';
      Serial.println("ROBOT STANDING");
      break;
  }
}

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);

  if (!SerialBT.begin("FRSBOT")) {
    Serial.println("Bluetooth start failed!");
  } else {
    Serial.println("Bluetooth started: FRSBOT");
  }

  Serial.println("\n==============================");
  Serial.println("       FRSBOT 12 DOF");
  Serial.println("    CONTINUOUS WALKING");
  Serial.println("==============================");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 12; i++) {

    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 500, 2400);

    delay(20);
  }

  delay(500);

  standRobot();

  Serial.println("\nROBOT READY");
  Serial.println("------------------------------");
  Serial.println("F = CONTINUOUS FORWARD");
  Serial.println("B = CONTINUOUS BACKWARD");
  Serial.println("L = LEFT TURN");
  Serial.println("R = RIGHT TURN");
  Serial.println("X = STOP AND STAND");
  Serial.println("S = NORMAL STANDING");
  Serial.println("Phone Bluetooth name: FRSBOT");
  Serial.println("------------------------------");
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  // While moving, the smooth movement functions call
  // checkStopCommand() repeatedly. While stopped, poll here.
  if (walkingMode == 'S' && !commandPending) {
    checkStopCommand();
  }

  // Run the currently selected gait cycle.
  bool cycleOK = true;

  if (walkingMode == 'F') {
    cycleOK = forwardWalkingCycle();
  }
  else if (walkingMode == 'B') {
    cycleOK = backwardWalkingCycle();
  }
  else if (walkingMode == 'L') {
    cycleOK = leftWalkingCycle();
  }
  else if (walkingMode == 'R') {
    cycleOK = rightWalkingCycle();
  }

  if (!cycleOK) {
    walkingMode = 'S';
  }

  // A new command interrupts the current gait. First return to
  // normal standing, then execute the requested command.
  if (stopRequested) {
    stopRequested = false;
    standRobot();
  }

  if (commandPending) {
    char command = pendingCommand;
    commandPending = false;
    pendingCommand = 0;
    executeCommand(command);
  }
}
